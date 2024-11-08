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

#pragma once

#include <project.h>
#include <stdint.h>

#define MOTOR1 0b01
#define MOTOR2 0b10
#define MOTOR_BOTH 0b11

#define FORWARD 1
#define BACKWARD 0

#define ERROR_PWM_NOT_ENABLED 0x30
#define ERROR_LIMIT 0x31

typedef struct {
    uint8 min_set, max_set, ratio_set;
    int32 tickMin, tickMax;
    int32 mDegMin, mDegMax;
    double ratio;
} Conversion;

int StartPWM(int motor);
void StopPWM(int motor);
int SetPWM(int motor, int16 pwm);
Conversion GetConversion(int motor);
void SetConvRatio(int motor, float ratio);
int SetConvMin(int motor, int32 tickMin, int32 mDegMin);
void SetConvMax(int motor, int32 tickMax, int32 mDegMax);
void SetEncOffset(int motor, int32 tick_offset);
void SetEncDir(int motor, uint8 dir);
int32 GetPotValue();
int32 GetEncValue();
int32 GetPosition(int motor);
uint8 GetLimitStatus();
int32 GetCurrentPWM(int motor);
void SetEncBound(uint8 lim_num, int32 value);

CY_ISR(Drive_Handler);

/* [] END OF FILE */
