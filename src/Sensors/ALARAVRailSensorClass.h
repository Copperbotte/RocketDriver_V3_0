#ifndef ALARAVRAILSENSORCLASS_H
#define ALARAVRAILSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "./States/SensorStates.hpp"
#include "./Base_Classes/Sensor.hpp"

class ALARAVRAIL_SENSOR : public Sensor
{
  private:
    // Per-class default values.  This should compile away, but I'm not sure... - Joe 2023 April 6
    const uint32_t _sampleRateSlowMode_Default = 1;        //the sample rate this given sensor will be read at
    const uint32_t _sampleRateMedMode_Default = 4;         //the sample rate this given sensor will be read at
    const uint32_t _sampleRateFastMode_Default = 25;        //the sample rate this given sensor will be read at
    const uint32_t _sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
    const sampleRateDefaults _SRD = sampleRateDefaults(_sampleRateSlowMode_Default, _sampleRateMedMode_Default, _sampleRateFastMode_Default, _sampleRateCalibrationMode_Default);

  public:
    void begin();                     // run in setup to get pins going
    void resetAll();                  // reset all configurable settings to defaults
    
    // constructor 1 - standard MCU external ADC read
    ALARAVRAIL_SENSOR(const idClass&setSensorID, uint8_t setADCinput, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Slow);
    // constructor 2 - simulated sensor object
};


#endif