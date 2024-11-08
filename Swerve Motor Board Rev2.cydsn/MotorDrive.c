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
#include "CAN_Stuff.h"

uint8  PWM1_enable = 0;
uint8  PWM2_enable = 0;

int32 PWM1_value = 0;
int32 PWM2_value = 0;

volatile uint8  PWM1_invalidate = 0;
volatile uint8  PWM2_invalidate = 0;

volatile int32 position1 = 0;
volatile int32 position2 = 0;
volatile int32 enc_value = 0;
volatile int32 pot_value = 0;

Conversion conv1 = {};
Conversion conv2 = {};

uint8 enc_dir = FORWARD;

uint8 limit1 = 0;
uint8 limit2 = 0;
uint8 bound_set1;
uint8 bound_set2;
int32 enc_lim_1;
int32 enc_lim_2;

int StartPWM(int motor) {
    if (motor & MOTOR1) {
        PWM1_enable = 1;
    }
    if (motor & MOTOR2) {
        PWM2_enable = 1;
    }
    return 0;
}

void StopPWM(int motor) {
    if (motor & MOTOR1) {
        PWM_Motor1_WriteCompare(0);
        PWM1_enable = 0;
    }
    if (motor & MOTOR2) {
        PWM_Motor2_WriteCompare(0);
        PWM2_enable = 0;
    }
}

// Sends PWM and Direction to the motor driver
// Also checks limits and sets PWM variables
// Accepts values -1024 - 1024
int SetPWM(int motor, int16 pwm) {
    int err = 0;
    if (motor & MOTOR1) {        
        if (PWM1_enable) {
            PWM1_invalidate = 0;
            
            if (pwm < 0) {
                Pin_Motor1_Dir_Write(BACKWARD);
                if (limit1) {
                    err = ERROR_LIMIT;
                    pwm = 0;
                }
            } else if (pwm > 0) {
                Pin_Motor1_Dir_Write(FORWARD);
                if (limit2) {
                    err = ERROR_LIMIT;
                    pwm = 0;
                }
            }
            
            PWM1_value = pwm;
            PWM_Motor1_WriteCompare(abs(pwm));
        } else err = ERROR_PWM_NOT_ENABLED;
    }
    if (motor & MOTOR2) {
        if (PWM2_enable) {
            PWM2_invalidate = 0;
            
            if (pwm < 0) {
                Pin_Motor2_Dir_Write(BACKWARD);
                // if (pot_value <= 0) {
                //     err = ERROR_LIMIT;
                //     pwm = 0;
                // }
            } else if (pwm > 0) {
                Pin_Motor2_Dir_Write(FORWARD);
                // if (pot_value >= 4095) {
                //     err = ERROR_LIMIT;
                //     pwm = 0;
                // }
            }
            
            PWM2_value = pwm;
            PWM_Motor2_WriteCompare(abs(pwm));
        } else err = ERROR_PWM_NOT_ENABLED;
    }
    
    return err;
}

int UpdateConversion(int motor) {
    if (motor & MOTOR1) {
        if (conv1.min_set && conv1.max_set && conv1.mDegMax != conv1.mDegMin) {
            conv1.ratio = (double) (conv1.mDegMax-conv1.mDegMin)/(conv1.tickMax-conv1.tickMin);
            conv1.ratio_set = 1;
        } else return 1;
    }
    if (motor & MOTOR2) {
        if (conv2.min_set && conv2.max_set && conv2.mDegMax != conv2.mDegMin) {
            conv2.ratio = (double) (conv2.mDegMax-conv2.mDegMin)/(conv2.tickMax-conv2.tickMin);
            conv2.ratio_set = 1;
        } else return 1;
    }
    return 0;
}

Conversion GetConversion(int motor) {
    if (motor == MOTOR1) return conv1;
    if (motor == MOTOR2) return conv2;
    return (Conversion) {};
}

void SetConvRatio(int motor, float ratio) {
    if (motor & MOTOR1) {
        conv1.ratio = ratio;
        conv1.ratio_set = 1;
    }
    if (motor & MOTOR2) {
        conv2.ratio = ratio;
        conv2.ratio_set = 1;
    }
}

int SetConvMin(int motor, int32 tickMin, int32 mDegMin) {
    if (motor & MOTOR1) {
        conv1.tickMin = tickMin;
        conv1.mDegMin = mDegMin;
        conv1.min_set = 1;
    }
    if (motor & MOTOR2) {
        conv2.tickMin = tickMin;
        conv2.mDegMin = mDegMin;
        conv2.min_set = 1;
    }
    return UpdateConversion(motor);
}

void SetConvMax(int motor, int32 tickMax, int32 mDegMax) {
    if (motor & MOTOR1) {
        conv1.tickMax = tickMax;
        conv1.mDegMax = mDegMax;
        conv1.max_set = 1;
    }
    if (motor & MOTOR2) {
        conv2.tickMax = tickMax;
        conv2.mDegMax = mDegMax;
        conv2.max_set = 1;
    }
    UpdateConversion(motor);
}

void SetEncOffset(int motor, int32 tick_offset) {
    if (motor & MOTOR1) {
        // QuadDec_Enc_WriteCounter(tick_offset);
    }
}
void SetEncDir(int motor, uint8 dir) {
    if (motor & MOTOR1) {
        enc_dir = dir;
    }
}

int UpdateEncValue() {
    uint32 val = 0; // QuadDec_Enc_ReadCounter();
    enc_value = enc_dir ? val : -val;
    return 0;
}

int UpdatePotValue() {
    int32 n = 100;
    int32 sum = 0;
    for (int i = 0; i < n; i++) {
        ADC_StartConvert();
        ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
        sum += ADC_GetResult16(0);
    }
    pot_value = sum/n;
    return 0;
}

uint8 GetLimitStatus() { return limit1 | (limit2 << 1); }
int32 GetPotValue() { return pot_value; }
int32 GetEncValue() { return enc_value; }
int32 GetPosition(int motor) {
    if (motor == MOTOR1) return position1;
    if (motor == MOTOR2) return position2;
    return 0;
}
int32 GetCurrentPWM(int motor) {
    if (motor == MOTOR1) return PWM1_value;
    if (motor == MOTOR2) return PWM2_value;
    return 0;
}

void UpdatePosition(int motor) {
    if (motor & MOTOR1) {
        if (conv1.ratio_set)
            position1 = (enc_value-conv1.tickMin) * conv1.ratio + conv1.mDegMin;
    }
    if (motor & MOTOR2) {
        if (conv2.ratio_set)
            position2 = (pot_value-conv2.tickMin) * conv2.ratio + conv2.mDegMin;
    }
}

void SetEncBound(uint8 lim_num, int32 value) {
    if (lim_num == 0) {
        bound_set1 = 1;
        enc_lim_1 = value;
    } else if (lim_num == 1) {
        bound_set2 = 1;
        enc_lim_2 = value;
    }
}

CY_ISR(Drive_Handler) {
    if (Pin_Limit1_Read() == 0) {
        if (limit1 == 0) {
            SetPWM(MOTOR1, 0);
            SendLimitAlert(1);
            if (bound_set1) SetEncOffset(MOTOR1, enc_lim_1);
        }
        limit1 = 1;
    } else limit1 = 0;
    
    if (Pin_Limit2_Read() == 0) {
        if (limit2 == 0) {
            SetPWM(MOTOR1, 0);
            SendLimitAlert(2);
            if (bound_set2) SetEncOffset(MOTOR1, enc_lim_2);
        }
        limit2 = 1;
    } else limit2 = 0;
    
    if (PWM1_invalidate == 20) SetPWM(MOTOR1, 0);
    else PWM1_invalidate++;
    
    if (PWM2_invalidate == 20) SetPWM(MOTOR2, 0);
    else PWM2_invalidate++;
    
    UpdatePotValue();
    UpdateEncValue();
    UpdatePosition(MOTOR_BOTH);
}

/* [] END OF FILE */
