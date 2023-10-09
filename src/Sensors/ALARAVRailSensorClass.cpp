#include "ALARAVRailSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>
#include <limits>

// Initializer 1
/* ALARAHP_SENSOR::ALARAHP_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off)
                : sensorID{setSensorID}, sensorNodeID{setSensorNodeID}, ADCinput{setADCinput}, sampleRateSlowMode_Default{setSampleRateSlowMode_Default}, sampleRateMedMode_Default{setSampleRateMedMode_Default}, sampleRateFastMode_Default{setSampleRateFastMode_Default}, linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default}, linConvCoef2_m_Default{setLinConvCoef2_m_Default}, linConvCoef2_b_Default{setLinConvCoef2_b_Default}, currentSampleRate{setCurrentSampleRate}, sensorState{setSensorState}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  linConvCoef1_m = linConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default;

  EMA_Enable = EMA_Default;
  alphaEMA = alphaEMA_Default;
  regressionSamples = regressionSamples_Default;
} */

// Initializer 2
ALARAVRAIL_SENSOR::ALARAVRAIL_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Slow)
    : Sensor{setSensorID, setSensorNodeID, setADCinput, _SRD,
        LinearMap{setLinConvCoef1_m_Default, setLinConvCoef1_b_Default, setLinConvCoef2_m_Default, setLinConvCoef2_b_Default},
        EMA{}, LinearRegression{}, IntegralError{}}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
  _currentSampleRate = setCurrentSampleRate;

// linConvCoef1_m = linConvCoef1_m_Default = setLinConvCoef1_m_Default;
// linConvCoef1_b = linConvCoef1_b_Default = setLinConvCoef1_b_Default;
// linConvCoef2_m = linConvCoef2_m_Default = setLinConvCoef2_m_Default;
// linConvCoef2_b = linConvCoef2_b_Default = setLinConvCoef2_b_Default;

// EMA_Enable = EMA_Enable_Default;
// alphaEMA = alphaEMA_Default;
// regressionSamples = regressionSamples_Default;
  sensorState = setSensorState;

  // This allows the code to be shared, but disables the clipping check.
// maxIntegralSum = maxIntegralSum_Default = std::numeric_limits<float>::max();
// minIntegralSum = minIntegralSum_Default = std::numeric_limits<float>::lowest();
}


void ALARAVRAIL_SENSOR::begin()
{
    if (nodeIDCheck)
    {
        pinMode(getADCinput(), INPUT);
    }
}

void ALARAVRAIL_SENSOR::resetAll()
{
  //
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  resetAllComponents();
// __linearMap.resetAll();
// __ema.resetAll();

// linConvCoef1_m = linConvCoef1_m_Default;
// linConvCoef1_b = linConvCoef1_b_Default;
// linConvCoef2_m = linConvCoef2_m_Default;
// linConvCoef2_b = linConvCoef2_b_Default;

// EMA_Enable = EMA_Enable_Default;
// alphaEMA = alphaEMA_Default;
}
/*
void ALARAVRAIL_SENSOR::read(ADC& adc)
{
    //Add in sample rate code here to check if a sensor is up to be read
    //This is also where alternate ADC sources would be used - I do have the RTD sensors over ITC right now
    //I'll have to change how it's written though, right now it's ADC* adc which is specific to Teensy MCU ADC
        if (getCurrentSampleRate() != 0)     //math says no divide by zero, use separate conditional for sample rate of 0
        {
        if (getTimer() >= (1000000/getCurrentSampleRate()))   // Divides 1 second in microseconds by current sample rate in Hz
            {
                    currentRawValue = adc.analogRead(getADCinput());
                    ////pullTimestamp = true;
                    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
                    /////linear conversions here, y = m*x + b
                    // This automatically stores converted value for the on board nodes
                    ////priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
                    ////currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;
                    linearConversion(currentRawValue); // Maps the voltage read by the ADC to the calibrated range.
                    //writeToRollingArray(convertedValueArray, currentConvertedValue);
                    exponentialMovingAverage(currentConvertedValue);
                    //accumulatedI_float();

                newSensorValueCheck_CAN = true;
                newSensorValueCheck_Log = true;
                newSensorConvertedValueCheck_CAN = true;
                //newSensorValueCheck = false;
                ////newConversionCheck = true;
                //Serial.println("newSensorinREADafter");
                //Serial.println(newSensorValueCheck);
                resetTimer();
                pullTimestamp = true;
            }
        
      }

}
*/
