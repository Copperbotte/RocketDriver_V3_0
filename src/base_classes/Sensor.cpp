#include "Sensor.hpp"

////////////////////////////////////////////////////////////////////////////////
// Common function for sensor state operations.
//     This function was the same for all sensors, so this is supplied as the 
// default. - Joe, 2023 April 6
void Sensor::stateOperations()
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
        __linearReg.setTimeStep(1); //timeStep in seconds - shitty hack to make it not brick to a nan from dividing by zero
    else
        __linearReg.setTimeStep(1/sampleRate);
}

// I dont like linearConversion being in here, but the simulated input doesn't use it in EXTSensorClass. - Joe, 2023 April 6
void Sensor::readRaw(ADC& adc)
{
    currentRawValue = adc.analogRead(getADCinput());
    newSensorValueCheck_CAN = true;
    newSensorValueCheck_Log = true;
    newSensorConvertedValueCheck_CAN = true; // This looks like it never does anything.

    ////pullTimestamp = true;
    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
    /////linear conversions here, y = m*x + b
    // This automatically stores converted value for the on board nodes
    //// priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
    //// currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;
    __linearMap.linearConversion(currentRawValue); // Maps the voltage read by the ADC to the calibrated range.
}

void Sensor::read(ADC& adc)
{
    //Add in sample rate code here to check if a sensor is up to be read
    //This is also where alternate ADC sources would be used - I do have the RTD sensors over ITC right now
    //I'll have to change how it's written though, right now it's ADC* adc which is specific to Teensy MCU ADC
    if (getCurrentSampleRate() != 0)     //math says no divide by zero, use separate conditional for sample rate of 0
    {
        if (__timer.getTimer() >= (1000000/getCurrentSampleRate()))   // Divides 1 second in microseconds by current sample rate in Hz
        {
            if (getADCtype() == TeensyMCUADC)
                readRaw(adc);
            if (getADCtype() == simulatedInput)
                readSim(adc);
            //writeToRollingArray(__linearReg.getConvertedValueArrayPtr(), __linearMap.getCurrentConvertedValue()); // Should this be on every sensor? is it slow? - Joe, 2023 April 6
            __linearReg.writeToRollingArray(__linearMap.getCurrentConvertedValue());
            __ema.exponentialMovingAverage(__linearMap.getCurrentConvertedValue());
            accumulatedI_float();
            //currentLinReg_a1 = linearRegressionLeastSquared_PID();

            //if (ID.getID
            //{
            //Serial.print("sensorID: ");
            //Serial.print(ID.getID());
            //Serial.print(", currentRawValue: ");
            //Serial.println(currentRawValue);
            //Serial.print(", currentConvertedValue: ");
            //Serial.println(currentConvertedValue); */
            //}
            //Serial.print("sensorID: ");
            //Serial.print(ID.getID());
            //Serial.print(", currentRawValue: ");
            //Serial.println(currentRawValue);
            //Serial.print(", currentRollingAverage: ");
            //Serial.println(getCurrentRollingAverage()); */
            //Serial.println("newSensorREADbefore");
            //Serial.println(newSensorValueCheck);
            ////newSensorValueCheck_CAN = true;
            ////newSensorValueCheck_Log = true;
            ////newSensorConvertedValueCheck_CAN = true;
                
            //newSensorValueCheck = false;
            ////newConversionCheck = true;
            //Serial.println("newSensorinREADafter");
            //Serial.println(newSensorValueCheck);
            __timer.resetTimer();
            pullTimestamp = true;
        }
    }
}


void LinearMap::linearConversion(uint32_t currentRaw)
{
    /////linear conversions here, y = m*x + b
    //if (newSensorValueCheck && newConversionCheck == false)

////////////////////////////////////////////////////////////////////////////////
    //     Disabled this condition, since linearConversion is only ever used in
    // read().  Maybe this should be inline too.  Only EXTDigitalDiffLCSensor
    // appears to use Secondary Calibration in its linearConverstion, but
    // nothing calls it.  Either set those values to 1 and 0, or disable the
    // line.  Alternatively, give that class a specific secondary converstion
    // function. - Joe, 2023 Apr 1
    // 
//if (newConversionCheck == false)
//{
    priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
    currentConvertedValue = linConvCoef1_m*currentRaw + linConvCoef1_b;    //Initial Calibration
    //currentConvertedValue = linConvCoef2_m*currentConvertedValue + linConvCoef2_b;    //Secondary Calibration
    newConversionCheck = true;

    /*      Serial.print("sensorID: ");
            Serial.print(ID.getID());
            Serial.print(", currentRawValue: ");
            Serial.println(currentRawValue);
            Serial.print(", currentConvertedValue: ");
            Serial.println(currentConvertedValue); */
//}
}

void LinearMap::linearConversion_WithSecondary(uint32_t currentRaw)
{
    /////linear conversions here, y = m*x + b
    //if (newSensorValueCheck && newConversionCheck == false)

////////////////////////////////////////////////////////////////////////////////
    //     Disabled this condition, since linearConversion is only ever used in
    // read().  Maybe this should be inline too.  Only EXTDigitalDiffLCSensor
    // appears to use Secondary Calibration in its linearConverstion, but
    // nothing calls it.  Either set those values to 1 and 0, or disable the
    // line.  Alternatively, give that class a specific secondary converstion
    // function. - Joe, 2023 Apr 1
    // 
//if (newConversionCheck == false)
//{
    priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
    currentConvertedValue = linConvCoef1_m*currentRaw + linConvCoef1_b;    //Initial Calibration
    currentConvertedValue = linConvCoef2_m*currentConvertedValue + linConvCoef2_b;    //Secondary Calibration
    newConversionCheck = true;

    /*      Serial.print("sensorID: ");
            Serial.print(ID.getID());
            Serial.print(", currentRawValue: ");
            Serial.println(currentRawValue);
            Serial.print(", currentConvertedValue: ");
            Serial.println(currentConvertedValue); */
//}
}

// This code, and the linear regression code were taken from the Thermocouple initially.
void EMA::exponentialMovingAverage(float EMA_Input)
{
    //function written to accept and return floats
    //alpha must be between 0 and 1, force overflows to max and min weights
    
    //Serial.print("alphaEMA");
    //Serial.println(alphaEMA);
    if (_EMA_Enable)  //only EMA if EMA_Enable bool is true
    {
        // bounds EMA between 0 and 1 for valid formula
        if (_alphaEMA >= 1) _alphaEMA = 1;
        else if (_alphaEMA <= 0) _alphaEMA = 0;

        //quick maffs
        _newEMAOutput = (_alphaEMA*EMA_Input) + ((1 - _alphaEMA)*(_priorEMAOutput));
        _priorEMAOutput = _newEMAOutput; // Isn't this backwards? priorEMAOutput is always newEMAOuptut. - Joe
    }
    else //EMA calc still runs this way but with no computation, just setting the values. Could possibly cut even this for performance.
    {
        _newEMAOutput = EMA_Input;
        _priorEMAOutput = _newEMAOutput;
    }
}

void LinearRegression::initializeLinReg(uint8_t arraySizeIn)
{
    if(!_enableLinearRegressionCalc) //only initializes the array if it wasn't already
    {
        _enableLinearRegressionCalc = true;
        //delete[]convertedValueArray;  //destroys the old version of the array
        //float convertedValueArray[arraySizeIn+3] = {};
        //rolling array setup
        _convertedValueArray[0] = {3};
        _convertedValueArray[1] = {3};
        //convertedValueArray[2] = {static_cast<float>(arraySizeIn)};
    }

/*   uint32_t arrayIndexFirstValueLinReg = 0;
  uint32_t arrayWrapSizeLinReg = 0;
  uint32_t arrayMostRecentPositionLinReg = 0;
  uint32_t regression_n = 0;
  uint32_t sizeInputArrayLinReg = 0;

  float sumX = 0;
  float sumY = 0;
  float sumXX = 0;
  float sumXY = 0;
  float denLeastSquare = 0;
  float a0LeastSquare = 0;
  float a1LeastSquare = 0; */
}

// 2023 Feb 18
// Current Authors: 
//     Joseph Kessler (joseph.b.kessler@gmail.com)
// 
////////////////////////////////////////////////////////////////////////////////
//     This function seems to smooth the recent PID by finding a slope using a
// linear regression.  I hope me cleaning this doesn't effect its output, 
// although this function's job should be a single variable using PID, or an 
// entirely other PID with a higher damping term.  Alternatively another PID to
// act as an EMA.
float LinearRegression::linearRegressionLeastSquared_PID()
{
    uint32_t arrayIndexFirstValueLinReg = 0;
    uint32_t arrayWrapSizeLinReg = 0;
    uint32_t arrayMostRecentPositionLinReg = 0;
    uint32_t regression_n = 0;
    uint32_t sizeInputArrayLinReg = 0;

    float sumX = 0;
    float sumY = 0;
    float sumXX = 0;
    float sumXY = 0;
    float denLeastSquare = 0;
    float a0LeastSquare = 0;
    float a1LeastSquare = 0;

    _timeStep = 0.01;

    // Offset recent by the size of the array to prevent negatives.
    int recent = _convertedValueArrayNextInd + _regressionSamples;

    for (int i=0; i < _regressionSamples; i++)
    {
        // Trace backwards through the array to find dx
        int di = (recent-i) % _regressionSamples;

        float dX = -i*_timeStep;
        sumX += dX;
        sumXX += dX*dX;
        sumY += _convertedValueArray[di];
        sumXY += _convertedValueArray[di] * dX;
    }
    
    // calculate the denominator term
    denLeastSquare = _regressionSamples*sumXX - (sumX * sumX);
    a1LeastSquare = ((_regressionSamples*sumXY) - (sumX * sumY))/denLeastSquare;
    return a1LeastSquare;
}

void Sensor::accumulatedI_float()
{
    float timeStepAccumI = __timer.getTimer()/float(1000000);
    __IErr.accumulatedI(timeStepAccumI, __linearMap.getCurrentConvertedValue(), __linearMap.getPriorConvertedValue());
}

void IntegralError::accumulatedI(float timeStepAccumI, float currentValue, float priorValue)
{
//float accumIfuncOutput = 0;
//float timeStepAccumI = 0;
    if (_enableIntegralCalc)
    {
        //timeStepAccumI = (currentTimestampSeconds - priorTimestampSeconds) + ((currentTimestampMicros - priorTimestampMicros)*1000000); //calculates timestep between samples in S
// timeStepAccumI = getTimer()/float(1000000);
        //timeStepAccumI = 0.01;
/*         Serial.print(" ID: ");
        Serial.print(ID.getID());
        Serial.print(" timer: ");
        Serial.println(timer,10);
        Serial.print(" timeStepAccumI: ");
        Serial.println(timeStepAccumI,10); */
        // trapazoid method for area under the curve using current and previous values as the end points
        /* Serial.print("currentInputValue");
        Serial.println(currentInputValue);
        Serial.print("previousInputValue");
        Serial.println(previousInputValue);
        Serial.print("accumIfuncOutput");
        Serial.println(accumIfuncOutput,10);
        Serial.print("timeStepAccumI");
        Serial.println(timeStepAccumI,10); */

        _currentIntegralSum += timeStepAccumI * (((currentValue - _targetValue) + (priorValue - _targetValue))/2);

        // Clamp output
        if (_currentIntegralSum >= _maxIntegralSum)
        {
            _currentIntegralSum = _maxIntegralSum;
        }
        else if (_currentIntegralSum <= _minIntegralSum)
        {
            _currentIntegralSum = _minIntegralSum;
        }
    }
}

