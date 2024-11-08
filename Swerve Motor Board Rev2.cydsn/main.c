/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "cyapicallbacks.h"
#include "CAN_Stuff.h"
#include "FSM_Stuff.h"
#include "MotorDrive.h"
#include "PositionPID.h"
#include "../CANLib/CANLibrary.h"

// LED stuff
volatile uint8 CAN_time_LED = 0;
volatile uint8 ERROR_time_LED = 0;

// UART stuff
char txData[TX_DATA_SIZE];

// CAN stuff
CANPacket can_recieve;
CANPacket can_send;

int main() { 
    Initialize();
    
    for(;;)
    {
        int err = 0;
        
        if (!PollAndReceiveCANPacket(&can_recieve)) {
            LED_CAN_Write(ON);
            CAN_time_LED = 0;
            PrintCanPacket(&can_recieve); // DEBUG
            err = ProcessCAN(&can_recieve, &can_send);
        }
        
        if (err) DisplayErrorCode(err);
        
        if (DBG_UART_SpiUartGetRxBufferSize()) {
            DebugPrint(DBG_UART_UartGetByte());
        }
        
        LED_DBG1_Write(!LED_DBG1_Read()); // visualize main loop rate
        LED_DBG2_Write(GetMode(MOTOR1) == MODE_UNINIT); // turn on when initialized
        CyDelay(1);
    }
}

void Initialize(void) {
    CyGlobalIntEnable;
    
    StartCAN(ReadDIP(), ReadDIP()+16);
    DBG_UART_Start();
    Timer_Periodic_Start();
    Timer_PID_Start();
    PWM_Motor1_Start();
    PWM_Motor2_Start();
    // QuadDec_Enc_Start();
    ADC_Start();
    
    sprintf(txData, "Address: %x & %x\r\n", GetAddress(MOTOR1), GetAddress(MOTOR2));
    Print(txData);
    
    isr_PID_StartEx(PID_Handler);
    isr_LED_StartEx(LED_Handler);
    isr_Drive_StartEx(Drive_Handler);
}

int ReadDIP() {
    return Status_Reg_DIP_Read() & 0x0F;
}

void DebugPrint(char input) {
    switch(input) {
        case 'm': // Mode
            sprintf(txData, "Address: %x & %x", GetAddress(MOTOR1), GetAddress(MOTOR2));
            Print(txData);
            Print(" Mode1: ");
            if (GetMode(MOTOR1) == MODE_UNINIT) Print("UNINIT");
            else if (GetMode(MOTOR1) == MODE_PWM_CTRL) Print("PWM");
            else if (GetMode(MOTOR1) == MODE_PID_CTRL) Print("PID");
            Print(" Mode2: ");
            if (GetMode(MOTOR2) == MODE_UNINIT) Print("UNINIT");
            else if (GetMode(MOTOR2) == MODE_PWM_CTRL) Print("PWM");
            else if (GetMode(MOTOR2) == MODE_PID_CTRL) Print("PID");
            break;
        // case 'a':
        //     SetMode(MOTOR1, MODE_PWM_CTRL);
        //     SetPWM(MOTOR1, 200);
        //     break;
        // case 'd':
        //     SetMode(MOTOR1, MODE_PWM_CTRL);
        //     SetPWM(MOTOR1, -200);
        //     break;
        // case 'w':
        //     SetMode(MOTOR2, MODE_PWM_CTRL);
        //     SetPWM(MOTOR2, 200);
        //     break;
        // case 's':
        //     SetMode(MOTOR2, MODE_PWM_CTRL);
        //     SetPWM(MOTOR2, -200);
        //     break;
        case 'p': // Position
            sprintf(txData, "Pos1:%li Pos2:%li PWM1:%li PWM2:%li", 
                GetPosition(MOTOR1), GetPosition(MOTOR2), 
                GetCurrentPWM(MOTOR1), GetCurrentPWM(MOTOR2));
            Print(txData);
            break;
        case 'o': // Raw sensor
            sprintf(txData, "Enc(%li) Pot(%li) Limits(", GetEncValue(), GetPotValue());
            Print(txData);
            PrintIntBin(GetLimitStatus());
            Print(")");
            break;
        case 'i': // PID Config
            sprintf(txData, "Motor1 Target(%li) P(%li) I(%li) D(%li)\r\nMotor2 Target(%li) P(%li) I(%li) D(%li)",
                GetPIDState(MOTOR1).target, GetPIDConfig(MOTOR1).kP, GetPIDConfig(MOTOR1).kI, GetPIDConfig(MOTOR1).kD,
                GetPIDState(MOTOR2).target, GetPIDConfig(MOTOR2).kP, GetPIDConfig(MOTOR2).kI, GetPIDConfig(MOTOR2).kD);
            Print(txData);
            break;
        case 'k': 
            ADC_StartConvert();
            ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
            PrintInt(ADC_GetResult16(0));
            // Print(" ");
            // PrintInt(ADC_GetResult16(1));
            // Print(" ");
            // PrintInt(ADC_GetResult16(2));
            break;
        case 'c': // Conversion
            sprintf(txData, "Motor1 TickMin(%li) TickMax(%li) mDegMin(%li) mDegMax(%li) ratio(%li)\r\n",
                GetConversion(MOTOR1).tickMin, GetConversion(MOTOR1).tickMax,
                GetConversion(MOTOR1).mDegMin, GetConversion(MOTOR1).mDegMax, (long) GetConversion(MOTOR1).ratio);
            Print(txData);
            sprintf(txData, "Motor2 TickMin(%li) TickMax(%li) mDegMin(%li) mDegMax(%li) ratio(%li)",
                GetConversion(MOTOR2).tickMin, GetConversion(MOTOR2).tickMax,
                GetConversion(MOTOR2).mDegMin, GetConversion(MOTOR2).mDegMax, (long) GetConversion(MOTOR2).ratio);
            Print(txData);
            break;
        // case '1': // PID init
        //     SetConvMin(MOTOR2, 650, 180000);
        //     SetConvMax(MOTOR2, 1350, -180000);
        //     
        //     SetkPosition(MOTOR2, 1);
        //     SetkIntegral(MOTOR2, 0);
        //     SetkDerivative(MOTOR2, 0);
        //     SetMode(MOTOR2, MODE_PID_CTRL);
        //     
        //     SetPIDTarget(MOTOR2, 0);
        //     Print("PID Initialized");
        //     break;
        // case '9': // PID dec
        //     SetPIDTarget(MOTOR2, GetPIDState(MOTOR2).target-10000);
        //     Print("PID target set to ");
        //     PrintInt(GetPIDState(MOTOR2).target);
        //     break;
        // case '0': // PID inc
        //     SetPIDTarget(MOTOR2, GetPIDState(MOTOR2).target+10000);
        //     Print("PID target set to ");
        //     PrintInt(GetPIDState(MOTOR2).target);
        //     break;
        default:
            Print("what");
            break;
    }
    Print("\r\n");
}

void PrintCanPacket(CANPacket* packet) {
    sprintf(txData, "ID %X DLC %X DATA", packet->id, packet->dlc);
    Print(txData);
    for(int i = 0; i < packet->dlc; i++ ) {
        sprintf(txData," %02X", packet->data[i]);
        Print(txData);
    }
    Print("\r\n");
}

void DisplayErrorCode(uint8 code) {    
    ERROR_time_LED = 0;
    LED_ERR_Write(ON);
    
    sprintf(txData, "Error %X: ", code);
    Print(txData);

    switch(code) {
        case ERROR_INVALID_PACKET:
            Print("Packet type not recognized\r\n");
            break;
        case ERROR_PWM_NOT_ENABLED:
            Print("PWM is not enabled\r\n");
            break;
        case ERROR_LIMIT:
            Print("At limit\r\n");
            break;
        case ERROR_WRONG_MODE:
            Print("Wrong mode\r\n");
            break;
        case ERROR_MODE_CHANGE:
            Print("Failed to change mode\r\n");
            break;
        case ERROR_INVALID_TTC:
            Print("Cannot send that data type\r\n");
            break;
        case ERROR_ESTOP:
            Print("ESTOP\r\n");
            break;
        default:
            Print(":(\r\n");
            break;
    }
}

CY_ISR(LED_Handler) {
    CAN_time_LED++;
    ERROR_time_LED++;
    
    if (ERROR_time_LED >= 10)
        LED_ERR_Write(OFF);
    if (CAN_time_LED >= 2)
        LED_CAN_Write(OFF);
}

/* [] END OF FILE */
