#include "EXTSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>
#include <InternalTemperature.h>

//#include <ADC_util.h>

//using std::string;


void MCUADCSetup(ADC& adc, ADC_REFERENCE refIn0, ADC_REFERENCE refIn1, uint8_t averagesIn0, uint8_t averagesIn1)
{ 
//Ideally get some conditionals here for which MCU it is so this is compatible at least also with Teensy LC

///// ADC0 /////
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 or ADC_REFERENCE::REF_EXT.
  //adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc.adc0->setReference(refIn0);
  adc.adc0->setAveraging(averagesIn0);                                    // set number of averages
  adc.adc0->setResolution(16);                                   // set bits of resolution
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed
  adc.adc0->recalibrate();

///// ADC1 /////
  adc.adc1->setReference(refIn1);
  adc.adc1->setAveraging(averagesIn1);                                    // set number of averages
  adc.adc1->setResolution(16);                                   // set bits of resolution
  adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed
  adc.adc1->recalibrate();

}


// Initializer 1
EXT_SENSOR::EXT_SENSOR(const idClass&setSensorID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, 
    float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0,
    float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off)
    : Sensor{setSensorID, setADCinput, sampleRateDefaults{setSampleRateSlowMode_Default, setSampleRateMedMode_Default, setSampleRateFastMode_Default, _SRD.sampleRateCalibrationMode_Default},
        LinearMap{setLinConvCoef1_m_Default, setLinConvCoef1_b_Default, setLinConvCoef2_m_Default, setLinConvCoef2_b_Default},
        EMA{}, LinearRegression{}, IntegralError{setMinIntegralSum_Default, setMaxIntegralSum_Default, false}},
      fluidSim{*setFluidSim}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  _currentSampleRate = setCurrentSampleRate;
  
  sensorState = SensorState::Fast;
}

// Initializer 2
EXT_SENSOR::EXT_SENSOR(const idClass&setSensorID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, ADCType setSensorSource)
    : Sensor{setSensorID, setADCinput, _SRD,
        LinearMap{}, EMA{}, LinearRegression{},
        IntegralError{setMinIntegralSum_Default, setMaxIntegralSum_Default, false},
        setSensorSource},
      fluidSim{*setFluidSim}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
  //temporarily default initialize the simulated sensors to a given rate
  //currentSampleRate = 200;
  sensorState = SensorState::Slow;
}

void EXT_SENSOR::begin()
{
    
    if (ID.getNodeIDCheck())
    {
        //rolling array setup
        __linearReg.initConvertedValueArray(3,3,static_cast<float>(__linearReg.getRegressionSamples()));
    }
    if (ID.getNodeIDCheck() && getADCtype() == TeensyMCUADC)
    {
        pinMode(getADCinput(), INPUT);
    }
}

void EXT_SENSOR::resetAll()
{
    sampleRateSlowMode = sampleRateSlowMode_Default;
    sampleRateMedMode = sampleRateMedMode_Default;
    sampleRateFastMode = sampleRateFastMode_Default;
    sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

    resetAllComponents();
}

void EXT_SENSOR::readSim(ADC& adc)
{
    float currentConvertedValue = fluidSim.analogRead(getADCinput());
    __linearMap._overrideValues(__linearMap.getPriorConvertedValue(), currentConvertedValue);
}
