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
#include "../CANLib/CANLibrary.h"

// CAN Errors (0x10-0x1F)
#define ERROR_ESTOP           0x10
#define ERROR_WRONG_MODE      0x11
#define ERROR_INVALID_MODE    0x12
#define ERROR_INVALID_TTC     0x13
#define ERROR_INVALID_PACKET  0x14

void StartCAN(uint8 addy1, uint8 addy2);
int ProcessCAN(CANPacket* receivedPacket, CANPacket* packetToSend);
int SendLimitAlert(uint8 status);
uint8 GetAddress(int motor);

/* [] END OF FILE */
    