#ifndef ALARABOARDCONTROLLERDEFINITIONS_H
#define ALARABOARDCONTROLLERDEFINITIONS_H
#include "./Controllers/ALARABoardControllerClass.h"
#include "./ALARApinDefines.h"

ALARAbuzzer buzzerr(ALARA_BUZZ);
// Library using default B000000 (A5-A0) i2c address, and default Wire @400kHz
PCA9685 ALARALEDExtPWM;

RGB_LED Led1 {0,1,2};
RGB_LED Led2 {3,4,5};
//ALARABoardController boardController(ALARALEDExtPWM, Led1, Led2, buzzerr);
ALARABoardController boardController(&ALARALEDExtPWM, ALARA_PWM_EXPANDER_OE, &Led1, &Led2, &buzzerr);

//ALARA HP channel array
//position zero is a null channel don't use
static uint8_t ALARA_HP_Array[3][11] = {
    {
        0,
        ALARA_HIGHPOWER_DIGITALOUT1,
        ALARA_HIGHPOWER_DIGITALOUT2,
        ALARA_HIGHPOWER_DIGITALOUT3,
        ALARA_HIGHPOWER_DIGITALOUT4,
        ALARA_HIGHPOWER_DIGITALOUT5,
        ALARA_HIGHPOWER_DIGITALOUT6,
        ALARA_HIGHPOWER_DIGITALOUT7,
        ALARA_HIGHPOWER_DIGITALOUT8,
        ALARA_HIGHPOWER_DIGITALOUT9,
        ALARA_HIGHPOWER_DIGITALOUT10
    },

    {
        0,
        ALARA_HIGHPOWER_PWMOUT1,
        ALARA_HIGHPOWER_PWMOUT2,
        ALARA_HIGHPOWER_PWMOUT3,
        ALARA_HIGHPOWER_PWMOUT4,
        ALARA_HIGHPOWER_PWMOUT5,
        ALARA_HIGHPOWER_PWMOUT6,
        ALARA_HIGHPOWER_PWMOUT7,
        ALARA_HIGHPOWER_PWMOUT8,
        ALARA_HIGHPOWER_PWMOUT9,
        ALARA_HIGHPOWER_PWMOUT10
    },

    {
        0,
        ALARA_HIGHPOWER_ANALOGREAD1,
        ALARA_HIGHPOWER_ANALOGREAD2,
        ALARA_HIGHPOWER_ANALOGREAD3,
        ALARA_HIGHPOWER_ANALOGREAD4,
        ALARA_HIGHPOWER_ANALOGREAD5,
        ALARA_HIGHPOWER_ANALOGREAD6,
        ALARA_HIGHPOWER_ANALOGREAD7,
        ALARA_HIGHPOWER_ANALOGREAD8,
        ALARA_HIGHPOWER_ANALOGREAD9,
        ALARA_HIGHPOWER_ANALOGREAD10
    }
};


#endif
