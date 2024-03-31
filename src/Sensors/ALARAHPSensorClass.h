#ifndef ALARAHPSENSORCLASS_H
#define ALARAHPSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "./States/SensorStates.hpp"
#include "./Base_Classes/Sensor.hpp"
//#include "ALARAUtilityFunctions.h"

class ALARAHP_SENSOR : public Sensor
{
private:
    // Per-class default values.  This should compile away, but I'm not sure... - Joe 2023 April 6
    static sampleRateDefaults _SRD(){
        uint32_t _sampleRateSlowMode_Default = 5;        //the sample rate this given sensor will be read at
        uint32_t _sampleRateMedMode_Default = 10;         //the sample rate this given sensor will be read at
        uint32_t _sampleRateFastMode_Default = 1000;        //the sample rate this given sensor will be read at
        uint32_t _sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
        return sampleRateDefaults{_sampleRateSlowMode_Default, _sampleRateMedMode_Default, _sampleRateFastMode_Default, _sampleRateCalibrationMode_Default};
    }

    elapsedMicros OffsetFunctimer;                      // timer for sensor timing operations
    float deenergizeOffset = 24;

public:
    void begin();                     // run in setup to get pins going
    void resetAll();                  // reset all configurable settings to defaults
    
    // constructor 1 - standard MCU external ADC read
    //ALARAHP_SENSOR(const idClass&setSensorID, uint8_t setADCinput, ADC* setADC, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);
    //ALARAHP_SENSOR(const idClass&setSensorID, uint8_t setADCinput, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);
    ALARAHP_SENSOR(const idClass&setSensorID, uint8_t setADCinput, const LinearMap&linearMap = LinearMap(), uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Fast);
    // constructor 2 - simulated sensor object

    float getDeengergizeOffsetValue(){return deenergizeOffset;}
    float getCurrentOutputValue(){return (deenergizeOffset - __ema.getEMAConvertedValue());}
    float getCurrentOutputValue(bool resetConvertedRead){if (resetConvertedRead) {__linearMap.setNewConversionCheck(false);} return (deenergizeOffset - __ema.getEMAConvertedValue());} //reads and clears new value bool

    void setDeenergizeOffset(ADC& adc, bool outputOverrideIn);
};


#endif