#include "ALARAHPSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>


// Initializer 1
ALARAHP_SENSOR::ALARAHP_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Fast)
    : Sensor{setSensorID, setSensorNodeID, setADCinput}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
  _currentSampleRate = setCurrentSampleRate;
  
  linConvCoef1_m = linConvCoef1_m_Default = setLinConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default = setLinConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default = setLinConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default = setLinConvCoef2_b_Default;

  EMA = EMA_Default;
  alphaEMA = alphaEMA_Default;
  regressionSamples = regressionSamples_Default;
  sensorState = setSensorState;
}


void ALARAHP_SENSOR::begin()
{
    if (nodeIDCheck)
    {
        pinMode(getADCinput(), INPUT);
    }
}

void ALARAHP_SENSOR::resetAll()
{
  //
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  linConvCoef1_m = linConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default;

  EMA = EMA_Default;
  alphaEMA = alphaEMA_Default;
}

void ALARAHP_SENSOR::read(ADC& adc)
{
    //Add in sample rate code here to check if a sensor is up to be read
    //This is also where alternate ADC sources would be used - I do have the RTD sensors over ITC right now
    //I'll have to change how it's written though, right now it's ADC* adc which is specific to Teensy MCU ADC
        if (getCurrentSampleRate() != 0)     //math says no divide by zero, use separate conditional for sample rate of 0
        {
        if (getTimer() >= (1000000/getCurrentSampleRate()))   // Divides 1 second in microseconds by current sample rate in Hz
            {
                    currentRawValue = adc.analogRead(getADCinput());
                    pullTimestamp = true;
                    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
                    /////linear conversions here, y = m*x + b
                    // This automatically stores converted value for the on board nodes
                    priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
                    currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;
                    //writeToRollingArray(convertedValueArray, currentConvertedValue);
                    exponentialMovingAverage();
                    //accumulatedI_float();

                newSensorValueCheck_CAN = true;
                newSensorValueCheck_Log = true;
                newSensorConvertedValueCheck_CAN = true;
                //newSensorValueCheck = false;
                newConversionCheck = true;
                //Serial.println("newSensorinREADafter");
                //Serial.println(newSensorValueCheck);
                resetTimer();
            }
        
      }

}

void ALARAHP_SENSOR::setDeenergizeOffset(ADC& adc, bool outputOverrideIn)
{
  // read the value but don't flag it as a new value for any messages. This is purely for ripping off a bunch of calibration reads.
  // only do this during output override so it gaurantees all outputs are full off
  if (outputOverrideIn)
  {
    if (OffsetFunctimer >= (2000))   // 500 Hz fixed, using separate timer to be independant from regular read function
      {
        currentRawValue = adc.analogRead(getADCinput());
        /////linear conversions here, y = m*x + b
        priorConvertedValue = currentConvertedValue;
        currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;
        exponentialMovingAverage();
        OffsetFunctimer = 0;
      }
  // while output override still has this running, update the deenergize offset
  deenergizeOffset = newEMAOutput;
  }
}

void ALARAHP_SENSOR::stateOperations()
{
    uint32_t sampleRate = 0;
    switch(sensorState)
    {
    case SensorState::Slow:   sampleRate = sampleRateSlowMode; break;
    case SensorState::Medium: sampleRate = sampleRateMedMode;  break;
    case SensorState::Fast:   sampleRate = sampleRateFastMode; break;
    case SensorState::Off:    sampleRate = 0; break;
    default: return;
    }

    setCurrentSampleRate(sampleRate);
    if(sensorState == SensorState::Off)
        timeStep = 1; //timeStep in seconds - shitty hack to make it not brick to a nan from dividing by zero
    else
        timeStep = 1/sampleRate;
}
