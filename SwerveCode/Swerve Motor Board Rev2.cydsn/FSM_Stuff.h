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
#include <stdint.h>

//Operation modes
#define MODE_UNINIT     0x0
#define MODE_PWM_CTRL   0x1
#define MODE_PID_CTRL   0x2

#define ERROR_MODE_CHANGE 0x20

int SetMode(int motor, int mode);
int GetMode(int motor);

/* [] END OF FILE */