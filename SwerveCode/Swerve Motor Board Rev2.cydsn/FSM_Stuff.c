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

#include "FSM_Stuff.h"
#include "MotorDrive.h"
#include "project.h"
#include "PositionPID.h"

int mode1, mode2;

int SetMode(int motor, int new_mode) {
    int err = 0;
    
    if (new_mode == MODE_UNINIT) {
        StopPID(motor);
        StopPWM(motor);
    } else if (new_mode == MODE_PWM_CTRL) {
        StopPID(motor);
        err = StartPWM(motor);
    } else if (new_mode == MODE_PID_CTRL) {
        err = StartPID(motor);
        if (!err)
            err = StartPWM(motor);
    }
    
    if (err) return ERROR_MODE_CHANGE;
    if (motor & MOTOR1) mode1 = new_mode;
    if (motor & MOTOR2) mode2 = new_mode;
    return 0;
}

int GetMode(int motor) {
    if (motor == MOTOR1) return mode1;
    if (motor == MOTOR2) return mode2;
    return -1;
}

/* [] END OF FILE */