#include "EXTDigitalDiffLCSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>

// Initializer 1
DIG_LC_SENSOR::DIG_LC_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput1, uint8_t setADCinput2, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, uint32_t setConversionSendRate_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off)
    : Sensor{setSensorID, setSensorNodeID, setADCinput1, sampleRateDefaults{setSampleRateSlowMode_Default, setSampleRateMedMode_Default, setSampleRateFastMode_Default, _SRD.sampleRateCalibrationMode_Default}},
    ADCinput2{setADCinput2}, fluidSim{*setFluidSim}, conversionSendRate_Default{setConversionSendRate_Default}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
  conversionSendRate = conversionSendRate_Default;
  _currentSampleRate = setCurrentSampleRate;

  linConvCoef1_m = linConvCoef1_m_Default = setLinConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default = setLinConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default = setLinConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default = setLinConvCoef2_b_Default;

  EMA_Enable = EMA_Enable_Default;
  alphaEMA = alphaEMA_Default;
  regressionSamples = regressionSamples_Default;
  maxIntegralSum = maxIntegralSum_Default = setMaxIntegralSum_Default;
  minIntegralSum = minIntegralSum_Default = setMinIntegralSum_Default;
  sensorState = setSensorState;
}


void DIG_LC_SENSOR::begin()
{
    
    if (nodeIDCheck)
    {
        //rolling array setup
        convertedValueArray[0] = {3};
        convertedValueArray[1] = {3};
        convertedValueArray[2] = {static_cast<float>(regressionSamples)};
    }
    if (nodeIDCheck && getADCtype() == TeensyMCUADC)
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

  linConvCoef1_m = linConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default;

  EMA_Enable = EMA_Enable_Default;
  alphaEMA = alphaEMA_Default;
  maxIntegralSum = maxIntegralSum_Default;
  minIntegralSum = minIntegralSum_Default;

}

void DIG_LC_SENSOR::readRaw(ADC& adc)
{
    currentRawValue = adc.analogRead(getADCinput());
    currentRawValue2 = adc.analogRead(getADCinput2());
    currentRawDiffValue = currentRawValue - currentRawValue2;
    newSensorValueCheck_CAN = true;
    newSensorValueCheck_Log = true;
    newSensorConvertedValueCheck_CAN = true;
    Serial.print(", currentDiffRawValue: ");
    Serial.println(currentRawDiffValue);

    //pullTimestamp = true;
    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
    /////linear conversions here, y = m*x + b
    // This automatically stores converted value for the on board nodes
    ////priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
    ////currentConvertedValue = linConvCoef1_m*currentRawDiffValue + linConvCoef1_b;
    linearConversion(currentRawDiffValue); // Maps the voltage read by the ADC to the calibrated range.
}
/*
void DIG_LC_SENSOR::read(ADC& adc)
{
    //Add in sample rate code here to check if a sensor is up to be read
    //This is also where alternate ADC sources would be used - I do have the RTD sensors over ITC right now
    //I'll have to change how it's written though, right now it's ADC* adc which is specific to Teensy MCU ADC
// if (getADCtype() == TeensyMCUADC)
//     {
        if (getCurrentSampleRate() != 0)     //math says no divide by zero, use separate conditional for sample rate of 0
        {
        if (getTimer() >= (1000000/getCurrentSampleRate()))   // Divides 1 second in microseconds by current sample rate in Hz
            {
                if (getADCtype() == TeensyMCUADC)
                {
                    currentRawValue = adc.analogRead(getADCinput());
                    currentRawValue2 = adc.analogRead(getADCinput2());
                    currentRawDiffValue = currentRawValue - currentRawValue2;
                    newSensorValueCheck_CAN = true;
                    newSensorValueCheck_Log = true;
                    newSensorConvertedValueCheck_CAN = true;
                Serial.print(", currentDiffRawValue: ");
                Serial.println(currentRawDiffValue);
                
                    //pullTimestamp = true;
                    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
                    /////linear conversions here, y = m*x + b
                    // This automatically stores converted value for the on board nodes
                    ////priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
                    ////currentConvertedValue = linConvCoef1_m*currentRawDiffValue + linConvCoef1_b;
                    linearConversion(currentRawDiffValue); // Maps the voltage read by the ADC to the calibrated range.
                }
                writeToRollingArray(convertedValueArray, currentConvertedValue);
                exponentialMovingAverage(currentConvertedValue);
                accumulatedI_float();
                //currentLinReg_a1 = linearRegressionLeastSquared_PID();

                //if (getSensorID() == 58)
                //{
                //Serial.print("sensorID: ");
                //Serial.print(getSensorID());
                //Serial.print(", currentRawValue: ");
                //Serial.println(currentRawValue);
                //Serial.print(", currentConvertedValue: ");
                //Serial.println(currentConvertedValue); 
                //}
                //Serial.print("sensorID: ");
                //Serial.print(getSensorID());
                //Serial.print(", currentRawValue: ");
                //Serial.println(currentRawValue);
                //Serial.print(", currentRollingAverage: ");
                //Serial.println(getCurrentRollingAverage()); 
                //Serial.println("newSensorREADbefore");
                //Serial.println(newSensorValueCheck);
                ////newSensorValueCheck_CAN = true;
                ////newSensorValueCheck_Log = true;
                ////newSensorConvertedValueCheck_CAN = true;
                //newSensorValueCheck = false;
                ////newConversionCheck = true;
                //Serial.println("newSensorinREADafter");
                //Serial.println(newSensorValueCheck);
                resetTimer();
                pullTimestamp = true;
            }
        }
//}

}
*/

// // This linear conversion has currentRawDiffValue present instead of currentRawValue.
// // Perhaps there's a way to consolodate both?  Maybe this should be a class on its own, with an input?
// // - Joe, 2023 April 1
// void DIG_LC_SENSOR::linearConversion()
// {
//     /////linear conversions here, y = m*x + b
//     //if (newSensorValueCheck && newConversionCheck == false)
//     if (newConversionCheck == false)
//     {
//     //priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
//     currentConvertedValue = linConvCoef1_m*currentRawDiffValue + linConvCoef1_b;    //Initial Calibration
//     currentConvertedValue = linConvCoef2_m*currentConvertedValue + linConvCoef2_b;    //Secondary Calibration
//     newConversionCheck = true;
//     }
// }
