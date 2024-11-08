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

#include <stdio.h>
#include "main.h"
#include "CAN_Stuff.h"
#include "FSM_Stuff.h"
#include "PositionPID.h"
#include "MotorDrive.h"
#include "../CANLib/CANLibrary.h"

uint8 address1, address2;

void StartCAN(uint8 addy1, uint8 addy2) {
    if (addy1 == 0) {
        address1 = DEVICE_SERIAL_TELEM_LOCALIZATION;
    } else address1 = addy1;
    if (addy2 == 0) {
        address2 = DEVICE_SERIAL_TELEM_LOCALIZATION;
    } else address2 = addy2;
    
    InitCAN(0x04, (int) address1, (int) address2);
}

uint8 GetAddress(int motor) {
    if (motor == MOTOR1) return address1;
    if (motor == MOTOR2) return address2;
    return 0;
}

//Reads from CAN FIFO and changes the state and mode accordingly
int ProcessCAN(CANPacket* receivedPacket, CANPacket* packetToSend) {  
    uint16 packageID = GetPacketID(receivedPacket);
    
    if (packageID == ID_ESTOP) {
        SetMode(MOTOR_BOTH, MODE_UNINIT);
        return ERROR_ESTOP;
    }
    
    uint8  motor_address = GetDeviceSerialNumber(receivedPacket);
    uint8  sender_DG = GetSenderDeviceGroupCode(receivedPacket);
    uint8  sender_SN = GetSenderDeviceSerialNumber(receivedPacket);
    
    int32 data = 0;
    int err = 0;
    int motor = 0;
    
    if (motor_address == address1) motor = MOTOR1;
    else if (motor_address == address2) motor = MOTOR2;
    else motor = MOTOR_BOTH; // assume broadcast
    
    switch(packageID) {
        case(ID_MOTOR_UNIT_MODE_SEL):
            data = GetModeFromPacket(receivedPacket);
            if (data == MOTOR_UNIT_MODE_PWM) {
                err = SetMode(motor, MODE_PWM_CTRL);
            } else if (data == MOTOR_UNIT_MODE_PID) {
                err = SetMode(motor, MODE_PID_CTRL);
            } else err = ERROR_INVALID_PACKET;
            break;
           
        case(ID_MOTOR_UNIT_PWM_DIR_SET):
            if (GetMode(motor) != MODE_PWM_CTRL) {
                err = ERROR_WRONG_MODE;
            } else {
                data = GetPWMFromPacket(receivedPacket);
                err = SetPWM(motor, data/32);
            }
            break;
            
        case(ID_MOTOR_UNIT_PID_P_SET):
            data = GetPFromPacket(receivedPacket);
            SetkPosition(motor, data);
            break;
        
        case(ID_MOTOR_UNIT_PID_I_SET):
            data = GetIFromPacket(receivedPacket);
            SetkIntegral(motor, data);
            break;
            
        case(ID_MOTOR_UNIT_PID_D_SET):
            data = GetDFromPacket(receivedPacket);
            SetkDerivative(motor, data);
            break;
            
        case(ID_MOTOR_UNIT_PID_POS_TGT_SET):
            data = GetPIDTargetFromPacket(receivedPacket);
            SetPIDTarget(motor, data);
            if (GetMode(motor) != MODE_PID_CTRL) err = ERROR_WRONG_MODE;
            break;
            
        case(ID_MOTOR_UNIT_ENC_PPJR_SET):
            data = GetEncoderPPJRFromPacket(receivedPacket);
            SetConvRatio(motor, 360.0*1000/data);
            break;
            
        case (ID_MOTOR_UNIT_POT_INIT_LO):
            SetConvMin(motor, 
                GetPotADCFromPacket(receivedPacket), 
                GetPotmDegFromPacket(receivedPacket));
            break;
            
        case (ID_MOTOR_UNIT_POT_INIT_HI):
            SetConvMax(motor, 
                GetPotADCFromPacket(receivedPacket), 
                GetPotmDegFromPacket(receivedPacket));
            break;
        
        case(ID_MOTOR_UNIT_ENC_INIT):
            // didn't bother checking the use_pot bit
            if(GetEncoderZeroFromPacket(receivedPacket)) {
                SetEncOffset(motor, 0);
            }
            SetEncDir(motor, GetEncoderDirectionFromPacket(receivedPacket));
            break;
            
        case(ID_MOTOR_UNIT_MAX_PID_PWM):
            SetPIDMaxPWM(motor, GetMaxPIDPWMFromPacket(receivedPacket));
            break;
            
        case(ID_MOTOR_UNIT_SET_ENCODER_BOUND):
            SetEncBound(GetLimSwNumFromPacket(receivedPacket), GetEncoderValueFromPacket(receivedPacket));
            break;
            
        case(ID_TELEMETRY_PULL):            
            switch(DecodeTelemetryType(receivedPacket))
            {
                case(PACKET_TELEMETRY_ANG_POSITION):
                    data = GetPosition(motor);
                    break;
                case(PACKET_TELEMETRY_ADC_RAW):
                    ADC_StartConvert();
                    ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
                    data = ADC_GetResult16(0);
                    break;
                case(PACKET_TELEMETRY_LIM_SW_STATE):
                    data = GetLimitStatus();
                    break;
                case(PACKET_TELEMETRY_CHIP_TYPE):
                    data = CHIP_TYPE_PSOC_CY8C4248AZI_L485;
                    break;
                default:
                    err = ERROR_INVALID_TTC;
                    break;
            }
            
            if (!err) {
                // Assemble and send packet
                AssembleTelemetryReportPacket(packetToSend, sender_DG, sender_SN, receivedPacket->data[3], data);
                packetToSend->data[2] = motor_address; // gross
                SendCANPacket(packetToSend);
            }
            break;

        default: //recieved Packet with non-valid ID
            // could be due to corruption, don't uninit
            return ERROR_INVALID_PACKET;
    }
    
    return err;
}

int SendLimitAlert(uint8 status) {
    CANPacket can_send;
    AssembleLimitSwitchAlertPacket(&can_send, DEVICE_GROUP_JETSON, 
    DEVICE_SERIAL_JETSON, status);
    
    return SendCANPacket(&can_send);
}


/* [] END OF FILE */