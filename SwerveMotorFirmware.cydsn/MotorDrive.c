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
#include <project.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "MotorDrive.h"

uint8_t invalidate = 0;     // TODO: make sure this isn't a motor specific variable
extern char txData[TX_DATA_SIZE];

int32_t currPWM_Drive = 0;
int32_t currPWM_Swivel = 0;


// Drive Motor, Motor 1 in Top Design
// takes between -255 and 255
void set_PWM_M1(int16_t compare, uint8_t disable_limit, uint8_t limitSW) {  //limitSw prev uint8
    #ifdef PRINT_PWM_COMMAND
    sprintf(txData, "PWM:%d disable_limit: %d\r\n",compare,disable_limit);
    UART_UartPutString(txData); 
    #endif
    //test
//    char out[32];
//    UART_UartPutString("compare val m1: ");
//    UART_UartPutString(itoa(compare, out, 10));
//    UART_UartPutString("\n\r");
        //show results
//        UART_UartPutString(itoa(Hum_val, out1, 10));
        //UART_UartPutString("\n\r");
//        UART_UartPutString(itoa(Temp_val, out1, 10));
    invalidate = 0;     
    if (compare < 0 && (!(limitSW & 0b01) || disable_limit) ) {
        Motor1Direction_Write(0);
        currPWM_Drive = compare;
        PWM_Motor1_WriteCompare(-compare);
    } else if (compare > 0 && (!(limitSW & 0b10) || disable_limit) ){
        Motor1Direction_Write(1);
        currPWM_Drive = compare;
        PWM_Motor1_WriteCompare(compare);
    } else {
        currPWM_Drive = 0;
        PWM_Motor1_WriteCompare(0);
    }
}

//  Swivel Motor, Motor 2 in Top Design
void set_PWM_M2(int16_t compare, uint8_t disable_limit, uint8_t limitSW) { 
    #ifdef PRINT_PWM_COMMAND
    sprintf(txData, "PWM:%d disable_limit: %d\r\n",compare,disable_limit);
    UART_UartPutString(txData); 
    #endif
    
    invalidate = 0;    
//    char out[32];
//    UART_UartPutString("compare val m2: ");
//    UART_UartPutString(itoa(compare, out, 10));
//    UART_UartPutString("\n\r");
    if (compare < 0 && (!(limitSW & 0b01) || disable_limit) ) {
        Motor2Direction_Write(0);
        currPWM_Swivel = compare;
        PWM_Motor2_WriteCompare(-compare);
    } else if (compare > 0 && (!(limitSW & 0b10) || disable_limit) ){
        Motor2Direction_Write(1);
        currPWM_Swivel = compare;
        PWM_Motor2_WriteCompare(compare);
    } else {
        currPWM_Swivel = 0;
        PWM_Motor2_WriteCompare(0);
    }
}

int16_t GetCurrentPWM_M1() {
    return currPWM_Drive;
}

int16_t GetCurrentPWM_M2() {
    return currPWM_Swivel;
}
/* [] END OF FILE */
