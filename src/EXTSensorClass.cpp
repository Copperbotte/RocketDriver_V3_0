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
EXT_SENSOR::EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off)
    : Sensor{setSensorID, setSensorNodeID, setADCinput}, fluidSim{*setFluidSim}, sampleRateSlowMode_Default{setSampleRateSlowMode_Default}, sampleRateMedMode_Default{setSampleRateMedMode_Default}, sampleRateFastMode_Default{setSampleRateFastMode_Default}
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
  maxIntegralSum = maxIntegralSum_Default = setMaxIntegralSum_Default;
  minIntegralSum = minIntegralSum_Default = setMinIntegralSum_Default;
  sensorState = SensorState::Fast;
}

// Initializer 2
EXT_SENSOR::EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, ADCType setSensorSource)
    : Sensor{setSensorID, setSensorNodeID, setADCinput, setSensorSource}, fluidSim{*setFluidSim}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
  //temporarily default initialize the simulated sensors to a given rate
  //currentSampleRate = 200;
  sensorState = SensorState::Slow;

  // I have no idea what these defaults are.  Is this constructor even used?
  linConvCoef1_m = linConvCoef1_m_Default = 0;
  linConvCoef1_b = linConvCoef1_b_Default = 0;
  linConvCoef2_m = linConvCoef2_m_Default = 0;
  linConvCoef2_b = linConvCoef2_b_Default = 0;

  EMA = EMA_Default;
  alphaEMA = alphaEMA_Default;
  regressionSamples = regressionSamples_Default;
  maxIntegralSum = maxIntegralSum_Default = setMaxIntegralSum_Default;
  minIntegralSum = minIntegralSum_Default = setMinIntegralSum_Default;
}

void EXT_SENSOR::begin()
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
        pinMode(getADCinput(), INPUT);
    }
}

void EXT_SENSOR::resetAll()
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
  maxIntegralSum = maxIntegralSum_Default;
  minIntegralSum = minIntegralSum_Default;
}

void EXT_SENSOR::read(ADC& adc)
{
    //Add in sample rate code here to check if a sensor is up to be read
    //This is also where alternate ADC sources would be used - I do have the RTD sensors over ITC right now
    //I'll have to change how it's written though, right now it's ADC* adc which is specific to Teensy MCU ADC
if (getADCtype() == TeensyMCUADC)
    {
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
                    writeToRollingArray(convertedValueArray, currentConvertedValue);
                    exponentialMovingAverage();
                    accumulatedI_float();
                    //currentLinReg_a1 = linearRegressionLeastSquared_PID();

                //if (getSensorID() == 58)
                //{
    /*             Serial.print("sensorID: ");
                Serial.print(getSensorID());
                Serial.print(", currentRawValue: ");
                Serial.println(currentRawValue);
                Serial.print(", currentConvertedValue: ");
                Serial.println(currentConvertedValue); */
                //}
    /*             Serial.print("sensorID: ");
                Serial.print(getSensorID());
                Serial.print(", currentRawValue: ");
                Serial.println(currentRawValue);
                Serial.print(", currentRollingAverage: ");
                Serial.println(getCurrentRollingAverage()); */
                //Serial.println("newSensorREADbefore");
                //Serial.println(newSensorValueCheck);
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

if (getADCtype() == simulatedInput)
    {
        if (getCurrentSampleRate() != 0)     //math says no divide by zero, use separate conditional for sample rate of 0
        {
        if (getTimer() >= (1000000/getCurrentSampleRate()))   // Divides 1 second in microseconds by current sample rate in Hz
            {
                
                    //currentRawValue = adc->analogRead(ADCinput);
                    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
                    /////linear conversions here, y = m*x + b
                    // This automatically stores converted value for the on board nodes
                    pullTimestamp = true;
                    priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
                    currentConvertedValue = fluidSim.analogRead(getADCinput());
                    writeToRollingArray(convertedValueArray, currentConvertedValue);
                    exponentialMovingAverage();
                    accumulatedI_float();
                    resetTimer();
            }
        }
    }
}

void EXT_SENSOR::stateOperations()
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

