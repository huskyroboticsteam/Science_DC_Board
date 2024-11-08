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

typedef struct {
    uint8 kP_set, kI_set, kD_set;
    int32 kP, kI, kD, maxPWM, maxIntegral;
} PID_Config;

typedef struct {
    int8 target_set;
    int32 target;
    int32 integral;
    int32 last_error;
} PID_State;

int StartPID(int motor);
void StopPID(int motor);
void SetkPosition(int motor, int32 kP);
void SetkIntegral(int motor, int32 kI);
void SetkDerivative(int motor, int32 kD);
void SetPIDMaxPWM(int motor, uint16 maxPWM);
void SetPIDTarget(int motor, int32 mDegs);
PID_State GetPIDState(int motor);
PID_Config GetPIDConfig(int motor);

CY_ISR(PID_Handler);

/* [] END OF FILE */
