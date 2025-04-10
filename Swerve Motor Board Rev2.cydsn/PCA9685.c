/* ========================================
 *
 * Driver for PCA9685
 * 
 * Name I2C Block in Top Design "I2C"
 *
 * Written by Dupri Chipman and Davis Sauer
 * 
 *
 * ========================================
*/

#include "project.h"
#include "PCA9685.h"

void pca_init(){
    I2C_Start();
    I2C_Enable();
    
    uint8 mode_buf[2] = {0,0b10010000}; // put to sleep (bit 4) to enable prescaler update
    writeBuffer(mode_buf, PCA9685ADDY);
    
    uint8 freq_buf[2] = {254, 121};     // write to register 254 the magic num 121 to set 50hz pwm
    writeBuffer(freq_buf, PCA9685ADDY);
    
    uint8 mode_buf2[2] = {0,0b10000000}; // unsleep to resume normal operation
    writeBuffer(mode_buf2, PCA9685ADDY);
}

/*Writes a single byte over i2c*/
void writeByte(uint8 chip_register, uint8 value){
     uint8 buf[2] = {chip_register, value};
     writeBuffer(buf, PCA9685ADDY);
}


/*Casting delayTime as uint8_t gets rid of the four left most significant bits*/
uint8 TurnOnTimeL(){
           int delayTime = DELAYTIME; /*This ammounts to 41 ticks before turn on*/
           //dutyCycle = (4095*dutyCycle)/100;
            delayTime = (4095*delayTime)/100 - 1;
        return delayTime;
}

/*Shifting to the right keeps the four most signifcant bits but gets rid of 8 least significant digits */
uint8 TurnOnTimeH(){
    int delayTime = DELAYTIME;
    delayTime = (4095*delayTime)/100 - 1;
    delayTime = delayTime >> 8;
        return delayTime;
}

uint8 TurnOffTimeL(float32 dutyCycle){
    int delayTime = DELAYTIME;
    int offTime=0;
    if(delayTime+((int)dutyCycle)<=100){
    delayTime = (4095*delayTime)/100 - 1;
    dutyCycle = (4095*dutyCycle)/100;
    offTime = delayTime + dutyCycle;
    } else {
    delayTime = (4095*delayTime)/100 - 1;
    dutyCycle = (4095*dutyCycle)/100;
    offTime = delayTime + dutyCycle - 4096;
    offTime = offTime;
    }
       return offTime;}

uint8 TurnOffTimeH(float32 dutyCycle){
    int delayTime = DELAYTIME;
    int offTime = 0; //initialized because program giving a warning
    if(delayTime+((int)dutyCycle)<=100){
    delayTime = (4095*delayTime)/100 - 1;
    dutyCycle = (4095*dutyCycle)/100;
    offTime = delayTime + dutyCycle;
    } else {
    delayTime = (4095*delayTime)/100 - 1;
    dutyCycle = (4095*dutyCycle)/100;
    offTime = delayTime + dutyCycle - offTime;
    }
       return offTime >> 8;}

uint32 writeBuffer(uint8 *buff, uint32_t PCA9685Address){
     uint32 status = TRANSFER_ERROR;
    
    (void) I2C_I2CMasterClearStatus();
         if( I2C_I2C_MSTR_NO_ERROR ==  I2C_I2CMasterWriteBuf(PCA9685Address, buff,
                                                            bufferSize, I2C_I2C_MODE_COMPLETE_XFER)){
     /* If I2C write started without errors, 
         * wait until I2C Master completes write transfer */
        while (0u == (I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT))
        {
        }
        
        /* Report transfer status */
        if (0u == (I2C_I2CMasterStatus() & I2C_I2C_MSTAT_ERR_XFER))
        {
            /* Check if all bytes were written */
            if (I2C_I2CMasterGetWriteBufSize() == bufferSize)
            {
                status = TRANSFER_CMPLT;
            }
        }
    }
    
    return status;}

//void setPWMFromBytes(int pinN, uint8 onTimeL, uint8 onTimeH, uint8 offTimeL, uint8 offTimeH){
//    uint8 registerNum = 0;
//    registerNum = 0x06 + pinN * 4;
//    uint8 buf[5] = {registerNum, onTimeL, onTimeH, offTimeL, offTimeH};
//    writeBuffer(buf, PCA9685ADDY);
//    
////    writeByte(registerNum, onTimeL);
////    writeByte(registerNum+1, onTimeH);
////    writeByte(registerNum+2, offTimeL);
////    writeByte(registerNum+3, offTimeH);
//}
void setPWMFromBytes(int pinN, uint8 onTimeL, uint8 onTimeH, uint8 offTimeL, uint8 offTimeH){
    uint8 registerNum = 0;
    registerNum = 0x06 + pinN * 4;
    
    writeByte(registerNum, onTimeL);
    writeByte(registerNum+1, onTimeH);
    writeByte(registerNum+2, offTimeL);
    writeByte(registerNum+3, offTimeH);
}

void setPWMFromDutyCycle(int pinN, float32 dutyCycle){
    uint8 registerNum = 0;
    registerNum = 0x06 + pinN * 4;
       
    writeByte(registerNum, TurnOnTimeL());
    writeByte(registerNum+1, TurnOnTimeH());
    writeByte(registerNum+2, TurnOffTimeL(dutyCycle));
    writeByte(registerNum+3, TurnOffTimeH(dutyCycle));
}

