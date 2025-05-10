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
#include "main.h"

int32_t PWM = 0;
int32_t kPosition = 0, kIntegral = 0, kDerivative = 0;
int32_t tickMax = 0, tickMin = 0, mDegMax = 0, mDegMin = 0;
double mDegPerTick;

int integral = 0;     //needs to be reset upon mode change
int lastError = 0; //needs to be reset upon mode change
int integralClamp = 500;
int32 maxPWM = 32767;

// double ratio;
// uint8 complete = 0;

uint8_t usingPot = 0;
uint8_t enabledPID = 0;

//encoder counter
extern int32_t counter;
extern uint8_t ignoreLimSw_M2;

extern char txData[TX_DATA_SIZE];

void ClearPIDProgress() {
    integral = 0;
    lastError = 0;
}
void DisablePID() {
    enabledPID = 0;
}
void EnablePID() {
    enabledPID = 1;
}
uint8_t PIDIsEnabled() {
    return(enabledPID);
}
void InitializePID() { //TODO: make work with both motors or ask if only swivel needed
    set_PWM_M2(0, 0, 0);
    ClearPIDProgress();
    DisablePID();
    lastError = 0;
}

void SetkPosition(int32_t kP){
    kPosition = kP;
}
void SetkIntegral(int32_t kI){
    kIntegral = kI;
}
void SetkDerivative(int32_t kD){
    kDerivative = kD;
}
int32_t GetPotVal() {
    int32_t n = 1000;
    int32_t sum = 0;
    for (int i = 0; i < n; i++) {
        ADC_Pot_StartConvert();
        ADC_Pot_IsEndConversion(ADC_Pot_WAIT_FOR_RESULT);
        sum += ADC_Pot_GetResult16(0);
    }    
    
    return sum/n;
}
void SetConversion(double conv) {
    mDegPerTick = conv;
}
double UpdateConversion() {
    if (mDegMin == mDegMax) return 0;
    mDegPerTick = (double) (tickMax-tickMin)/(mDegMax-mDegMin);
    return mDegPerTick;
}
int32_t GetkPosition(){
    return kPosition;
}
int32_t GetkIntegral(){
    return kIntegral;
}
int32_t GetkDerivative(){
    return kDerivative;
}
double GetConversion(){
    return mDegPerTick;
}
void setUsingPot(uint8_t pot) {
    usingPot = pot;
}
void SetMaxPIDPWM(uint16_t setValue){
    maxPWM = setValue;
}
int32_t GetMaxPIDPWM(){
    return maxPWM;
}
void setTickMin(int32_t val) { tickMin = val; }
void setTickMax(int32_t val) { tickMax = val; }
void setmDegMin(int32_t val) { mDegMin = val; }
void setmDegMax(int32_t val) { mDegMax = val; }

int32_t GetPositionmDeg() {
    if (mDegPerTick == 0.0)
        return(0);
    
    if (usingPot) 
        return (GetPotVal()-tickMin) * mDegPerTick + mDegMin;

    // return QuadDec_GetCounter() * mDegPerTick;
    return 0 * mDegPerTick;
}
void SetPosition(int32_t mDegs) {
        //TODO: Make Potentiometer Compatible
        PWM = Position_PID(mDegs);
        
        //Max Power clamp
        if(PWM > maxPWM){
            set_PWM_M2(maxPWM, ignoreLimSw_M2, Status_Reg_Switches_Read());   
        } else if(PWM < -maxPWM) {
            set_PWM_M2(-maxPWM, ignoreLimSw_M2, Status_Reg_Switches_Read());
        } else {
            set_PWM_M2(PWM, ignoreLimSw_M2, Status_Reg_Switches_Read());   
        }
}

int32_t Position_PID(int32 targetmDeg){
    if(!PIDIsEnabled()){
        return(0);
    }
    
    int32 current = GetPositionmDeg();
    int32 error = targetmDeg - current;
    
    //if within tolerance exit
    if(error <= 50 && error >= -50) {
      return(0);
    }
    
    integral = integral + error;
    
    //integral clamp
    if (integral > integralClamp) {
        integral = integralClamp;   
    }
    if (integral < -integralClamp) {
        integral = -integralClamp;
    }
    
    int derivative = error - lastError;
    int PWMOut = error*kPosition/10 + integral*kIntegral/10 + derivative*kDerivative/10;
    lastError = error;
    
    #ifdef PRINT_PID_DEBUG
        sprintf(txData,"c:%d, P:%d, I%d, D:%d, Out:%d", current, error, integral, derivative, PWMOut);
        UART_UartPutString(txData);   
    #endif
 
    return (PWMOut);
}

/* [] END OF FILE */
