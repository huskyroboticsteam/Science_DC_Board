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

/* [] END OF FILE */
#define ID_SET_LIM_SW_ENC_BOUND 0x12

#include "main.h"
#include "../CANLib/CANLibrary.h"
#include "Motor_Unit_CAN.h"
#include "Motor_Unit_FSM.h"
#include "MotorDrive.h"
#include "PositionPID.h"

#include <stdlib.h>

#ifdef RGB_LED_ARRAY
#include "LED_Array.h"
extern const uint32 StripLights_CLUT[ ];
extern uint8_t address;
#endif

extern uint8_t bound_set1;
extern uint8_t bound_set2;
extern int32_t enc_Count_Lim1; //TODO 
extern int32_t enc_Count_Lim2;
extern int32_t counter;

extern int16_t nextPWM_M1;
extern int16_t nextPWM_M2;
extern int32_t millidegreeTarget_M1;
extern int32_t millidegreeTarget_M2;

void SendEncoderData (CANPacket *packetToSend){
    AssembleTelemetryReportPacket(packetToSend, DEVICE_GROUP_JETSON, DEVICE_SERIAL_JETSON, 
        PACKET_TELEMETRY_ANG_POSITION, GetPositionmDeg());
    SendCANPacket(packetToSend);
}

static int32_t GetEncCountFromPacket(const CANPacket *packet);

static uint8_t GetLimitSwNumFromPacket(const CANPacket *packet);

//Reads from CAN FIFO and changes the state and mode accordingly
void NextStateFromCAN(CANPacket *receivedPacket, CANPacket *packetToSend) {
    uint16_t packageID = ReadCAN(receivedPacket);
    uint8_t motor_serial = GetDeviceSerialNumber(receivedPacket); //get which motor
    uint8_t sender_DG = GetSenderDeviceGroupCode(receivedPacket);
    uint8_t sender_SN = GetSenderDeviceSerialNumber(receivedPacket);

    //Motor 1: Drive
    if (motor_serial == address) {
        switch(packageID){
//            char out[32];
//            UART_UartPutString("package id : ");
//            UART_UartPutString(itoa(packageID, out, 10));
//            UART_UartPutString("\n\r");
            case(ID_MOTOR_UNIT_MODE_SEL):
                if(GetModeFromPacket(receivedPacket) == MOTOR_UNIT_MODE_PWM) {
                    set_PWM_M1(0, 0, 0);
                    #ifdef RGB_LED_ARRAY
                    StripLights_MemClear(StripLights_BLACK);
                    StripLights_Pixel(0, 0, get_color_packet(0,0,255));
                    StripLights_Trigger(1);
                    #endif
                    SetModeTo(MOTOR_UNIT_MODE_PWM);
                    SetStateTo(CHECK_CAN);
                }
                else if (GetModeFromPacket(receivedPacket) == MOTOR_UNIT_MODE_PID) {
                    InitializePID();
                    #ifdef RGB_LED_ARRAY
                    StripLights_MemClear(StripLights_BLACK);
                    StripLights_Pixel(1, 0, get_color_packet(0,0,255));
                    StripLights_Trigger(1);
                    #endif
                    SetModeTo(MOTOR_UNIT_MODE_PID);
                    SetStateTo(CHECK_CAN);
                } else {
                    GotoUninitState();
                }
                break;
                
            case(ID_MOTOR_UNIT_PWM_DIR_SET):    //TODO: Verify modifications are valid
                if(GetMode() == MOTOR_UNIT_MODE_PWM){
                    //SetStateTo(SET_PWM);        //NEED NEW FLAG
                    SetStateTo(SET_PWM_M1);
                    nextPWM_M1 = GetPWMFromPacket(receivedPacket);
                } else {
                    SetStateTo(QUEUE_ERROR);
                    DisplayErrorCode(MOTOR_ERROR_WRONG_MODE);
                }
                break;
            /*
            case(ID_MOTOR_UNIT_MAX_JNT_REV_SET):
                break;
            */
            case(ID_MOTOR_UNIT_ENC_INIT):
//                if(GetMode() == MOTOR_UNIT_MODE_PID){ //turn off and clear PID Loop if encoder is reinit while running
//                    set_PWM_M1(0, 0, 0);
//                    ClearPIDProgress();
//                    DisablePID();
//                }
                if(GetEncoderZeroFromPacket(receivedPacket)) {
                    //QuadDec_SetCounter(0);
                    counter = 0;
                }
                // if(GetEncoderDirectionFromPacket(receivedPacket)) {
                //     SetEncoderDirReverse();
                // } else {
                //     SetEncoderDirDefault();
                // }
                setUsingPot(0);
                SetStateTo(CHECK_CAN);
                break;
            
            // Common Packets 
            case(ID_ESTOP):
                set_PWM_M1(0, 0, 0);
                GotoUninitState();
                #ifdef RGB_LED_ARRAY
                StripLights_MemClear(StripLights_BLACK);
                StripLights_Pixel(0, 0, get_color_packet(0,0,255));
                StripLights_Trigger(1);
                #endif
                break;
            
            case(ID_TELEMETRY_PULL):
                AssembleChipTypeReportPacket(packetToSend, GetSenderDeviceGroupCode(receivedPacket),
                    GetSenderDeviceSerialNumber(receivedPacket));
                SendCANPacket(packetToSend);
                SetStateTo(CHECK_CAN);
                break;

                
                /*
            case(ID_LED_COLOR):
                break;
                */
                
                
            default://for 0xFF/no packets or Non recognized Packets
                
//                if(GetMode() == MOTOR_UNIT_MODE_PID){ //need to check if values set;
//                    SetStateTo(CALC_PID);
//                } else if(GetMode() == MOTOR_UNIT_MODE_PWM){ //skips PWM state to not reset saftey timer
//                    SetStateTo(CHECK_CAN);
//                } else {
//                    SetStateTo(CHECK_CAN);
//                }
                SetStateTo(CHECK_CAN);
                
                //recieved Packet with Non Valid ID
                if(packageID != NO_NEW_CAN_PACKET) {
                    DisplayErrorCode(MOTOR_ERROR_INVALID_PACKET);
                }
                break;
        }
        
    } else if (motor_serial == (address + 16)) {  //swivel
        switch(packageID){
            case(ID_MOTOR_UNIT_MODE_SEL):
                if(GetModeFromPacket(receivedPacket) == MOTOR_UNIT_MODE_PWM) {
                    set_PWM_M2(0, 0, 0);
                    #ifdef RGB_LED_ARRAY
                    StripLights_MemClear(StripLights_BLACK);
                    StripLights_Pixel(0, 0, get_color_packet(0,0,255));
                    StripLights_Trigger(1);
                    #endif
                    SetModeTo(MOTOR_UNIT_MODE_PWM);
                    SetStateTo(CHECK_CAN);
                }
                else if (GetModeFromPacket(receivedPacket) == MOTOR_UNIT_MODE_PID) {
                    InitializePID();
                    #ifdef RGB_LED_ARRAY
                    StripLights_MemClear(StripLights_BLACK);
                    StripLights_Pixel(1, 0, get_color_packet(0,0,255));
                    StripLights_Trigger(1);
                    #endif
                    SetModeTo(MOTOR_UNIT_MODE_PID);
                    SetStateTo(CHECK_CAN);
                } else {
                    GotoUninitState();
                }
                break;
                
            case(ID_MOTOR_UNIT_PWM_DIR_SET):    //TODO: Verify modifications are valid
                if(GetMode() == MOTOR_UNIT_MODE_PWM){
                    //SetStateTo(SET_PWM);        //NEED NEW FLAG
                    SetStateTo(SET_PWM_M2);
                    nextPWM_M2 = GetPWMFromPacket(receivedPacket);
                } else {
                    SetStateTo(QUEUE_ERROR);
                    DisplayErrorCode(MOTOR_ERROR_WRONG_MODE);
                }
                break;
                
            case(ID_MOTOR_UNIT_PID_P_SET):
                SetkPosition(GetPFromPacket(receivedPacket));
                PositionConstIsSet();
                SetStateTo(CHECK_CAN);
                break;
            
            case(ID_MOTOR_UNIT_PID_I_SET):
                SetkIntegral(GetIFromPacket(receivedPacket));
                IntegralConstIsSet();
                SetStateTo(CHECK_CAN);
                break;
            
            case(ID_MOTOR_UNIT_PID_D_SET):
                SetkDerivative(GetDFromPacket(receivedPacket));
                DerivativeConstIsSet();
                SetStateTo(CHECK_CAN);
                break;
            
            case(ID_MOTOR_UNIT_PID_POS_TGT_SET):
                if(GetMode() == MOTOR_UNIT_MODE_PID && PIDconstsSet()) {//&& PID values set
                    EnablePID();
                    millidegreeTarget_M2 = GetPIDTargetFromPacket(receivedPacket);
                    SetStateTo(CALC_PID);
                } else {
                    SetStateTo(QUEUE_ERROR);
                    DisplayErrorCode(MOTOR_ERROR_WRONG_MODE);
                }
                break;
            
            case(ID_MOTOR_UNIT_ENC_PPJR_SET):
                SetConversion(360.0*1000/GetEncoderPPJRFromPacket(receivedPacket));
                SetStateTo(CHECK_CAN);
                break;
            /*
            case(ID_MOTOR_UNIT_MAX_JNT_REV_SET):
                break;
            */

            case (ID_MOTOR_UNIT_POT_INIT_LO):
                //printf("pot init low: %0llX",(uint64_t) receivedPacket->data);
                setTickMin(GetPotADCFromPacket(receivedPacket));
                setmDegMin(GetPotmDegFromPacket(receivedPacket));
                UpdateConversion();
                setUsingPot(1);
                SetStateTo(CHECK_CAN);
                break;
                
            case (ID_MOTOR_UNIT_POT_INIT_HI):
                //printf("pot init hi: %0llX",(uint64_t) receivedPacket->data);
                setTickMax(GetPotADCFromPacket(receivedPacket));
                setmDegMax(GetPotmDegFromPacket(receivedPacket));
                UpdateConversion();
                setUsingPot(1);
                SetStateTo(CHECK_CAN);
                break;
            case(ID_MOTOR_UNIT_ENC_INIT):
                if(GetMode() == MOTOR_UNIT_MODE_PID){ //turn off and clear PID Loop if encoder is reinit while running
                    set_PWM_M2(0, 0, 0);
                    ClearPIDProgress();
                    DisablePID();
                }
                if(GetEncoderZeroFromPacket(receivedPacket)) {
                    counter = 0;
                    //QuadDec_SetCounter(0);                     
                }
                SetStateTo(CHECK_CAN);
                break;
            case(ID_MOTOR_UNIT_MAX_PID_PWM):
                SetMaxPIDPWM(GetMaxPIDPWMFromPacket(receivedPacket));
                SetStateTo(CHECK_CAN);
                break;    
            
            case(ID_SET_LIM_SW_ENC_BOUND) : //Set Limit Switch Encoder Bound
            
                if (GetLimitSwNumFromPacket(receivedPacket) == 0) { //lim1
                    bound_set1 = 1;
                    enc_Count_Lim1 = GetEncCountFromPacket(receivedPacket);
                } else if (GetLimitSwNumFromPacket(receivedPacket) == 1) {
                    bound_set2 = 1;
                    enc_Count_Lim2 = GetEncCountFromPacket(receivedPacket);
                }
                SetStateTo(CHECK_CAN);
                break;
            
            // case(ID_GPIO_BOARD_ADC_READ):
            //     PackIntIntoDataMSBFirst()
            //     AssembleTelemetryReportPacket(packetToSend, DEVICE_GROUP_JETSON, DEVICE_SERIAL_JETSON, 
            //     PACKET_TELEMETRY_ANG_POSITION, GetPositionmDeg());
            //     SendCANPacket(packetToSend);
            //     SetStateTo(CHECK_CAN);
            //     break;
            
            // Common Packets 
            case(ID_ESTOP):
                set_PWM_M2(0, 0, 0);
                GotoUninitState();
                #ifdef RGB_LED_ARRAY
                StripLights_MemClear(StripLights_BLACK);
                StripLights_Pixel(0, 0, get_color_packet(0,0,255));
                StripLights_Trigger(1);
                #endif
                break;
            
            case(ID_TELEMETRY_PULL):
                switch (DecodeTelemetryType(receivedPacket)) {
                    case(PACKET_TELEMETRY_ADC_RAW):
                        AssembleTelemetryReportPacket(packetToSend, sender_DG, sender_SN, 
                                                      PACKET_TELEMETRY_ADC_RAW, GetPotVal());
                    default:
                        AssembleChipTypeReportPacket(packetToSend, sender_DG, sender_SN);
                    break;
                }
                
                SendCANPacket(packetToSend);
                SetStateTo(CHECK_CAN);
                break;
               
                /*
            case(ID_LED_COLOR):
                break;
                */
                
            default://for 0xFF/no packets or Non recognized Packets
                
                if(GetMode() == MOTOR_UNIT_MODE_PID){ //need to check if values set;
                    SetStateTo(CALC_PID);
                } else if(GetMode() == MOTOR_UNIT_MODE_PWM){ //skips PWM state to not reset saftey timer
                    SetStateTo(CHECK_CAN);
                } else {
                    SetStateTo(CHECK_CAN);
                }
                
                //recieved Packet with Non Valid ID
                if(packageID != NO_NEW_CAN_PACKET) {
                    DisplayErrorCode(MOTOR_ERROR_INVALID_PACKET);
                }
                break;
        }
    }
}

static int32_t GetEncCountFromPacket(const CANPacket *packet) {
    return DecodeBytesToIntMSBFirst(packet->data, 2, 5);
}

static uint8_t GetLimitSwNumFromPacket(const CANPacket *packet) {
    return packet->data[1];
}
    
    //} else if (motor_serial == (getSerialAddress() + 1)) {
