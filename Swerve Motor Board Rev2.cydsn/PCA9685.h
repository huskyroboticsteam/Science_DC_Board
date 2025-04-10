#include "project.h"

#define PCA9685ADDY (0x40)

/* Transfer statuses */
#define TRANSFER_CMPLT (0x00u)
#define TRANSFER_ERROR      (0xFFu)

#define DELAYTIME 1

#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7
#define PIN8 8
#define PIN9 9
#define PIN10 10
#define PIN11 11
#define PIN12 12
#define PIN13 13
#define PIN14 14
#define PIN15 15
#define bufferSize 2

 /*Prototype*/
    void setPWMFromDutyCycle(int Ledn, float32 dutyCycle);
    void setPWMFromBytes(int pinN, uint8 onTimeL, uint8 onTimeH, uint8 offTimeL, uint8 offTimeH);
    uint8 TurnOnTimeL();
    uint8 TurnOnTimeH();
    uint8 TurnOffTimeH(float32 dutyCycle);
    uint8 TurnOffTimeL(float32 dutyCycle);
    uint32 writeBuffer(uint8 buff[],uint32_t PCA9685Address);
    void writeByte(uint8 chip_register, uint8 value);
    void pca_init();