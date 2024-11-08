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
#ifndef MotorDrive
    #define MotorDrive
#endif

#include <stdio.h>

void set_PWM_M1(int16_t compare, uint8_t disable_limit, uint8_t limitSW);
void set_PWM_M2(int16_t compare, uint8_t disable_limit, uint8_t limitSW);

int16_t GetCurrentPWM_M1();
int16_t GetCurrentPWM_M2();


/* [] END OF FILE */
