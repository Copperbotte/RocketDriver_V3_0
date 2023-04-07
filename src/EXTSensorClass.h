#ifndef EXTSENSORCLASS_H
#define EXTSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "States/SensorStates.hpp"
#include "Base_Classes/Sensor.hpp"
#include "fluidSystemSimulation.h"
#include "ALARAUtilityFunctions.h"


//Declaring setup of the ADC itself for main to find it
void MCUADCSetup(ADC& adc, ADC_REFERENCE refIn0, ADC_REFERENCE refIn1, uint8_t averagesIn0, uint8_t averagesIn1);

class EXT_SENSOR : public Sensor
{
private:
    // Per-class default values.  This should compile away, but I'm not sure... - Joe 2023 April 6
    const uint32_t _sampleRateSlowMode_Default = 1;        //the sample rate this given sensor will be read at
    const uint32_t _sampleRateMedMode_Default = 10;         //the sample rate this given sensor will be read at
    const uint32_t _sampleRateFastMode_Default = 200;        //the sample rate this given sensor will be read at
    const uint32_t _sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
    // Per-class default values.  This should compile away, but I'm not sure... - Joe 2023 April 6
    const sampleRateDefaults _SRD = sampleRateDefaults(_sampleRateSlowMode_Default, _sampleRateMedMode_Default, _sampleRateFastMode_Default, _sampleRateCalibrationMode_Default);
  
  FluidSystemSimulation &fluidSim;

  public:
    void begin();                     // run in setup to get pins going
    void resetAll();                  // reset all configurable settings to defaults
    void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    
    // constructor 1 - standard MCU external ADC read
    EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);
    // constructor 2 - simulated sensor object
    EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, ADCType setSensorSource = simulatedInput);

    // Access functions defined in place
    
    // Why does this instance have a nodeIDCheck?
//float getLinRegSlope(){if(nodeIDCheck){currentLinReg_a1 = linearRegressionLeastSquared_PID();} return currentLinReg_a1;}
    //float getLinRegSlope(){currentLinReg_a1 = linearRegressionLeastSquared_PID(); return currentLinReg_a1;}
};


#endif