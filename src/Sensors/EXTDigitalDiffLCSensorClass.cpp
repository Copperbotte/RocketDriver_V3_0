#include "EXTDigitalDiffLCSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>

// Initializer 1
DIG_LC_SENSOR::DIG_LC_SENSOR(const idClass&setSensorID, uint8_t setADCinput1, uint8_t setADCinput2, FluidSystemSimulation* setFluidSim, const sampleRateDefaults&SRD, uint32_t setConversionSendRate_Default, const LinearMap&linearMap, float setMaxIntegralSum_Default, float setMinIntegralSum_Default, uint32_t setCurrentSampleRate, SensorState setSensorState)
  : Sensor{setSensorID, setADCinput1, SRD,
    linearMap, EMA{}, LinearRegression{}, IntegralError{setMinIntegralSum_Default, setMaxIntegralSum_Default, false}},
    ADCinput2{setADCinput2}, fluidSim{*setFluidSim}, conversionSendRate_Default{setConversionSendRate_Default}
{
    // setting stuff to defaults at initialization
    sampleRateSlowMode = sampleRateSlowMode_Default;
    sampleRateMedMode = sampleRateMedMode_Default;
    sampleRateFastMode = sampleRateFastMode_Default;
    sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
    conversionSendRate = conversionSendRate_Default;
    _currentSampleRate = setCurrentSampleRate;

    sensorState = setSensorState;
}


void DIG_LC_SENSOR::begin()
{
    
    if (ID.getNodeIDCheck())
    {
        //rolling array setup
        __linearReg.initConvertedValueArray(3,3,static_cast<float>(__linearReg.getRegressionSamples()));
// convertedValueArray[0] = {3};
// convertedValueArray[1] = {3};
// convertedValueArray[2] = {static_cast<float>(regressionSamples)};
    }
    if (ID.getNodeIDCheck() && getADCtype() == TeensyMCUADC)
    {
        pinMode(getADCinput() , INPUT);
        pinMode(getADCinput2(), INPUT);
    }
}

void DIG_LC_SENSOR::resetAll()
{
  //
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
  conversionSendRate = conversionSendRate_Default;

  resetAllComponents();
}

uint32_t DIG_LC_SENSOR::readRaw(ADC& adc)
{
    currentRawValue = adc.analogRead(getADCinput());
    currentRawValue2 = adc.analogRead(getADCinput2());
    currentRawDiffValue = currentRawValue - currentRawValue2;
    newSensorValueCheck_CAN = true;
    newSensorValueCheck_Log = true;
    newSensorConvertedValueCheck_CAN = true;
    Serial.print(", currentDiffRawValue: ");
    Serial.println(currentRawDiffValue);

    return currentRawDiffValue;

    //return __linearMap.linearConversion(currentRawDiffValue); // Maps the voltage read by the ADC to the calibrated range.
}
