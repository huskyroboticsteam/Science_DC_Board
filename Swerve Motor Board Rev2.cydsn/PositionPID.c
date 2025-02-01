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

//PID varaibles
#include <project.h>
#include "PositionPID.h"
#include "MotorDrive.h"
#include "FSM_Stuff.h"

PID_Config PID1 = {.maxIntegral=500, .maxPWM=1023};
PID_Config PID2 = {.maxIntegral=500, .maxPWM=1023};
volatile PID_State PID1_state = {};
volatile PID_State PID2_state = {};

uint8 PID1_enable, PID2_enable;

int StartPID(int motor) {
    int err = 0;
    if (motor & MOTOR1) {
        if (PID1.kP_set && PID1.kI_set && PID1.kD_set && GetConversion(MOTOR1).ratio_set) {
            PID1_state.integral = 0;
            PID1_state.last_error = 0;
            PID1_enable = 1;
        } else err = 1;
    }
    if (motor & MOTOR2) {
        if (PID2.kP_set && PID2.kI_set && PID2.kD_set && GetConversion(MOTOR2).ratio_set) {
            PID2_state.integral = 0;
            PID2_state.last_error = 0;
            PID2_enable = 1;
        } else err = 1;
    }
    return err;
}

void StopPID(int motor) {
    if (motor & MOTOR1) {
        PID1_enable = 0;
        PID1_state.target_set = 0;
        SetPWM(MOTOR1, 0);
    }
    if (motor & MOTOR2) {
        PID2_enable = 0;
        PID2_state.target_set = 0;
        SetPWM(MOTOR2, 0);
    }
}

void SetkPosition(int motor, int32 kP) {
    if (motor & MOTOR1) {
        PID1.kP = kP;
        PID1.kP_set = 1;
    }
    if (motor & MOTOR2) { 
        PID2.kP = kP;
        PID2.kP_set = 1;
    }
}
void SetkIntegral(int motor, int32 kI) {
    if (motor & MOTOR1) {
        PID1.kI = kI;
        PID1.kI_set = 1;
    }
    if (motor & MOTOR2) { 
        PID2.kI = kI;
        PID2.kI_set = 1;
    }
}
void SetkDerivative(int motor, int32 kD) {
    if (motor & MOTOR1) {
        PID1.kD = kD;
        PID1.kD_set = 1;
    }
    if (motor & MOTOR2) { 
        PID2.kD = kD;
        PID2.kD_set = 1;
    }
}

void SetPIDMaxPWM(int motor, uint16 maxPWM) {
    if (motor & MOTOR1) PID1.maxPWM = maxPWM;
    if (motor & MOTOR2) PID2.maxPWM = maxPWM;
}

PID_Config GetPIDConfig(int motor) {
    if (motor == MOTOR1) return PID1;
    if (motor == MOTOR2) return PID2;
    return (PID_Config) {};
}
PID_State GetPIDState(int motor) {
    if (motor == MOTOR1) return PID1_state;
    if (motor == MOTOR2) return PID2_state;
    return (PID_State) {};
}


void SetPIDTarget(int motor, int32 mDegs) {
    if (motor & MOTOR1) {
        PID1_state.target = mDegs;
        PID1_state.target_set = 1;
    } if (motor & MOTOR2) {
        PID2_state.target = mDegs;
        PID2_state.target_set = 1;
    }
}

int PID_Update(int motor) {
    int error;
    volatile PID_State* state;
    PID_Config* pid;
    if (motor == MOTOR1) {        
        error = PID1_state.target - GetPosition(MOTOR1);
        
        state = &PID1_state;
        pid = &PID1;
    } else if (motor == MOTOR2) {        
        error = PID2_state.target - GetPosition(MOTOR2);

        state = &PID2_state;
        pid = &PID2;
    } else return 1;
    
    //if within tolerance exit
    // TODO: FIX MAGIC NUMBERS FOR ERROR TOLERANCE!!!!
    if(error <= 50 && error >= -50) {
        error = 0;
    }
    
    int integral = state->integral;
    integral += error;
    
    //integral clamp
    if (integral >  pid->maxIntegral) integral =  pid->maxIntegral;
    if (integral < -pid->maxIntegral) integral = -pid->maxIntegral;
    
    int derivative = error - state->last_error;
    int32 new_pwm = error*pid->kP/10 + integral*pid->kI/10 + derivative*pid->kD/10;
    state->last_error = error;
    state->integral = integral;
    
    //Max Power clamp
    if(new_pwm > pid->maxPWM){
        new_pwm = pid->maxPWM;
    } else if(new_pwm < -pid->maxPWM) {
        new_pwm = -pid->maxPWM;
    }
    
    //if within tolerance exit
    // TODO: FIX MAGIC NUMBERS FOR ERROR TOLERANCE!!!!
    if(new_pwm <= 1000 && new_pwm >= -1000) {
        new_pwm = 0;
    }
    
    return SetPWM(motor, new_pwm);
}

CY_ISR(PID_Handler) {
    if (PID1_enable && PID1_state.target_set) PID_Update(MOTOR1);
    if (PID2_enable && PID2_state.target_set) PID_Update(MOTOR2);
}

/* [] END OF FILE */
