#include "Sensor.hpp"

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
            Serial.print(getSensorID());
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
            Serial.print(getSensorID());
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
    if (EMA_Enable)  //only EMA if EMA_Enable bool is true
    {
        // bounds EMA between 0 and 1 for valid formula
        if (alphaEMA >= 1) alphaEMA = 1;
        else if (alphaEMA <= 0) alphaEMA = 0;

        //quick maffs
        newEMAOutput = (alphaEMA*EMA_Input) + ((1 - alphaEMA)*(priorEMAOutput));
        priorEMAOutput = newEMAOutput; // Isn't this backwards? priorEMAOutput is always newEMAOuptut. - Joe
    }
    else //EMA calc still runs this way but with no computation, just setting the values. Could possibly cut even this for performance.
    {
        newEMAOutput = EMA_Input;
        priorEMAOutput = newEMAOutput;
    }
}

void LinearRegression::initializeLinReg(uint8_t arraySizeIn)
{
    if(!enableLinearRegressionCalc) //only initializes the array if it wasn't already
    {
        enableLinearRegressionCalc = true;
        //delete[]convertedValueArray;  //destroys the old version of the array
        //float convertedValueArray[arraySizeIn+3] = {};
        //rolling array setup
        convertedValueArray[0] = {3};
        convertedValueArray[1] = {3};
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
    
    sumX = 0;
    sumY = 0;
    sumXX = 0;
    sumXY = 0;
    denLeastSquare = 0;
    a0LeastSquare = 0;
    a1LeastSquare = 0;
    
    
    // Version of linear regression simplified for finding the recent slope for a PID controller
    // assumes fixed time steps, time is X, controller variable Y
    // !!!!! - Function is built to expect arrays in format of:
    // !!!!! - index[0] = first index with a value entry
    // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
    // !!!!! - index[2] = size of value entries
    // Not sure if I've updated the math to use the first value in the numerical part instead of just manually subtracting to make it work for when it's 3
    arrayIndexFirstValueLinReg = static_cast<uint32_t>(convertedValueArray[0]+0.5);
    arrayMostRecentPositionLinReg = static_cast<uint32_t>(convertedValueArray[1]+0.5);
    sizeInputArrayLinReg = static_cast<uint32_t>(convertedValueArray[2]+0.5);
    
    // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
    if (sizeInputArrayLinReg < regressionSamples)
        regression_n = sizeInputArrayLinReg;
    else regression_n = regressionSamples;
    // determine the overwrap value, if any
    //arrayWrapSizeLinReg =  (-1) * ((arrayMostRecentPositionLinReg - regression_n) - 1); //old array method
    arrayWrapSizeLinReg =  regression_n - ((arrayMostRecentPositionLinReg) - (arrayIndexFirstValueLinReg) + 1);

    // calculate the sum terms;
    // 
        //Serial.print("timeStep: ");
        //Serial.println(timeStep);
    timeStep = 0.01;
    //dont think I need below with new methods
    /*     if (arrayWrapSizeLinReg <= 0)    // when there is no wrap required, calculated value will be zero or negative. Set to zero.
        {
        arrayWrapSizeLinReg = 0;
        } */
        //Serial.print("overwrap after zero set: ");
        //Serial.println(arrayWrapSizeLinReg);
    if (arrayWrapSizeLinReg > 0)  //only true if there are enough array values to use to wrap the end of the array
    {
        for (int i = arrayMostRecentPositionLinReg; i > (arrayIndexFirstValueLinReg - 1); i--)
        {
            float dX = (i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep;
            sumX += dX;
            sumXX += dX*dX;
            sumY += convertedValueArray[i];
            sumXY += convertedValueArray[i] * dX;
        /*       Serial.print("DOES THIS EVER HAPPEN1: ");
            Serial.print(i);
            Serial.print(" : ");
            Serial.println((i - (arrayMostRecentPositionLinReg - regression_n + 1)));
        */
        }

        for (int i = (sizeInputArrayLinReg + arrayIndexFirstValueLinReg - 1); i > (sizeInputArrayLinReg + arrayIndexFirstValueLinReg - 1 - arrayWrapSizeLinReg); i--)
        {
            float dX = (i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3)*timeStep;
            sumX += dX;
            sumXX += dX*dX;
            sumY += convertedValueArray[i];
            sumXY += (convertedValueArray[i] * dX);
        /*       Serial.print("DOES THIS EVER HAPPEN2: ");
            Serial.print(i);
            Serial.print(" : ");
            Serial.println((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3));
        */
        }
    }
    else
    {
        for (int i = arrayMostRecentPositionLinReg; i > (arrayMostRecentPositionLinReg - regression_n); i--)
        {
            float dX = (i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep;
            sumX += dX;
            sumXX += dX*dX;
            sumY += convertedValueArray[i];
            sumXY += convertedValueArray[i] * dX;
        /*       Serial.print("DOES THIS EVER HAPPEN: ");
            Serial.print(i);
            Serial.print(" : ");
            Serial.println((i - (arrayMostRecentPositionLinReg - regression_n + 1)));
        */    
        }
    }
    /*   Serial.print("sumX: ");
    Serial.println(sumX,8);
    Serial.print("sumY: ");
    Serial.println(sumY,8);
    Serial.print("sumXX: ");
    Serial.println(sumXX,8);
    Serial.print("sumXY: ");
    Serial.println(sumXY,8); */

    // calculate the denominator term
    denLeastSquare = regression_n*sumXX - (sumX * sumX);
    //Serial.print("den: ");
    //Serial.println(denLeastSquare,5);
    // calculate the a1 term, which is the slope
    a1LeastSquare = ((regression_n*sumXY) - (sumX*sumY))/denLeastSquare;
    // calculate the a0 term, which is the linear offset
    // NOT USED IN PID VERSION
    // a0LeastSquare = ((sumXX*sumY) - (sumXY*sumX))/denLeastSquare;
    return a1LeastSquare;
}

void Sensor::accumulatedI_float()
{
    float timeStepAccumI = getTimer()/float(1000000);
    accumulatedI(timeStepAccumI, currentConvertedValue, priorConvertedValue);
}

void IntegralError::accumulatedI(float timeStepAccumI, float currentValue, float priorValue)
{
//float accumIfuncOutput = 0;
//float timeStepAccumI = 0;
    if (enableIntegralCalc)
    {
        //timeStepAccumI = (currentTimestampSeconds - priorTimestampSeconds) + ((currentTimestampMicros - priorTimestampMicros)*1000000); //calculates timestep between samples in S
// timeStepAccumI = getTimer()/float(1000000);
        //timeStepAccumI = 0.01;
/*         Serial.print(" ID: ");
        Serial.print(getSensorID());
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

        currentIntegralSum += timeStepAccumI * (((currentValue - targetValue) + (priorValue - targetValue))/2);

        // Clamp output
        if (currentIntegralSum >= maxIntegralSum)
        {
          currentIntegralSum = maxIntegralSum;
        }
        else if (currentIntegralSum <= minIntegralSum)
        {
          currentIntegralSum = minIntegralSum;
        }
    }
}

