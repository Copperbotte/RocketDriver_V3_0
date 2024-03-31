#ifndef EXTDIGITALDIFFLCSENSORCLASS_H
#define EXTDIGITALDIFFLCSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "./States/SensorStates.hpp"
#include "./Base_Classes/Sensor.hpp"
#include "./fluidSystemSimulation.h"
#include "./ALARAUtilityFunctions.h"

class DIG_LC_SENSOR : public Sensor
{
private:
    uint8_t ADCinput2;               //the input that will be read for this sensor that will get used in the ADC read main loop

    // Per-class default values.  This should compile away, but I'm not sure... - Joe 2023 April 6
    static sampleRateDefaults _SRD(){
        uint32_t _sampleRateSlowMode_Default = 1;        //the sample rate this given sensor will be read at
        uint32_t _sampleRateMedMode_Default = 10;         //the sample rate this given sensor will be read at
        uint32_t _sampleRateFastMode_Default = 200;        //the sample rate this given sensor will be read at
        uint32_t _sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
        return sampleRateDefaults{_sampleRateSlowMode_Default, _sampleRateMedMode_Default, _sampleRateFastMode_Default, _sampleRateCalibrationMode_Default};
    }

    const uint32_t conversionSendRate_Default = 100;        //the sample rate this given sensor will be read at
    uint32_t conversionSendRate;        //the sample rate this given sensor will be read at
    uint32_t currentRawValue2{};               // holds the current value for the sensor
    uint32_t currentRawDiffValue{};               // holds the current value for the sensor
  
    FluidSystemSimulation &fluidSim;

public:
    void begin();                     // run in setup to get pins going
    void resetAll();                  // reset all configurable settings to defaults
    uint32_t readRaw(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object

    // constructor 1 - standard MCU external ADC read
    DIG_LC_SENSOR(const idClass&setSensorID, uint8_t setADCinput1, uint8_t setADCinput2, FluidSystemSimulation* setFluidSim, const sampleRateDefaults&SRD = _SRD(), uint32_t setConversionSendRate_Default = 100, const LinearMap&linearMap = LinearMap(), float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);

    // Access functions defined in place
    uint32_t getADCinput2(){return ADCinput2;}
    uint32_t getCurrentRawValue(bool input1, bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} if (!input1) {return currentRawValue;} else{return currentRawValue2;};} //reads and clears new value bool
};


#endif