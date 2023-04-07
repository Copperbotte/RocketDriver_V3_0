
#ifndef BASECLASS_SENSORCLASS_H_
#define BASECLASS_SENSORCLASS_H_

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "States/SensorStates.hpp"
#include "Timer.hpp"
//#include "fluidSystemSimulation.h"
//#include "ALARAUtilityFunctions.h"
//#pragma once
//#pragma GCC diagnostic ignored "-Wreorder" // go away cringe

// enum for holding ADC input types, may not use this way
enum ADCType
{
    TeensyMCUADC, //built in ADC
    ADS1258,  //not in use yet
    ADS1263,  //not in use yet
    simulatedInput, //for simulated sensor inputs
};
const ADCType ADCType_Default = ADCType::TeensyMCUADC;  //default source here is Teensy ADC

////////////////////////////////////////////////////////////////////////////////
// Timer-like classes that I currently only know are used with Sensor.

////////////////////////////////////////////////////////////////////////////////
// Linear map class for converting ADC outputs into a physical quantity.
// - Joe, 2023 April 1
class LinearMap
{
protected:
    float currentConvertedValue{};
    float priorConvertedValue{};

    // Base calibration coefficients
    float linConvCoef1_m_Default;
    float linConvCoef1_b_Default;
    float linConvCoef1_m;
    float linConvCoef1_b;

    // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef2_m_Default;
    float linConvCoef2_b_Default;
    float linConvCoef2_m;
    float linConvCoef2_b;

    bool newConversionCheck = false;                      // Is the current raw value a new read that hasn't been sent yet? 

public:
    // Initializes the LinearMap with no mapping.
    LinearMap():
        linConvCoef1_m_Default{0}, linConvCoef1_b_Default{1},
        linConvCoef1_m{0},         linConvCoef1_b{1},
        linConvCoef2_m_Default{0}, linConvCoef2_b_Default{1},
        linConvCoef2_m{0},         linConvCoef2_b{1}
        {}
    
    // // Initializes the LinearMap with no mapping on the secondary.
    // LinearMap(float setLinConvCoef1_m_Default, float setLinConvCoef1_b_Default):
    //     linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default},
    //     linConvCoef1_m{        setLinConvCoef1_m_Default}, linConvCoef1_b{        setLinConvCoef1_b_Default},
    //     linConvCoef2_m_Default{0},linConvCoef2_b_Default{1},
    //     linConvCoef2_m{0},        linConvCoef2_b{1}
    //     {}
    // 
    // // Initializes the LinearMap with both mappings.
    // LinearMap(float setLinConvCoef1_m_Default, float setLinConvCoef1_b_Default, float setLinConvCoef2_m_Default, float setLinConvCoef2_b_Default):
    //     linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default},
    //     linConvCoef1_m{        setLinConvCoef1_m_Default}, linConvCoef1_b{        setLinConvCoef1_b_Default},
    //     linConvCoef2_m_Default{setLinConvCoef2_m_Default}, linConvCoef2_b_Default{setLinConvCoef2_b_Default},
    //     linConvCoef2_m{        setLinConvCoef2_m_Default}, linConvCoef2_b{        setLinConvCoef2_b_Default}
    //     {}
    // 
    // // Copy constructor. 
    // LinearMap(const LinearMap &Other):
    //     linConvCoef1_m_Default{Other.linConvCoef1_m_Default}, linConvCoef1_b_Default{Other.linConvCoef1_b_Default},
    //     linConvCoef1_m{        Other.linConvCoef1_m_Default}, linConvCoef1_b{        Other.linConvCoef1_b_Default},
    //     linConvCoef2_m_Default{Other.linConvCoef2_m_Default}, linConvCoef2_b_Default{Other.linConvCoef2_b_Default},
    //     linConvCoef2_m{        Other.linConvCoef2_m_Default}, linConvCoef2_b{        Other.linConvCoef2_b_Default}
    //     {}

    // Maps ADC read value to the calibrated range using linConvCoef.
    // Accepts currentRawValue, or currentRawDiffValue.
    void linearConversion(uint32_t currentRaw); 
    void linearConversion_WithSecondary(uint32_t currentRaw); // Maps ADC read value to the calibrated range using linConvCoef, using both calibration steps.

    float getCurrentConvertedValue(){return currentConvertedValue;}
    //float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newSensorConvertedValueCheck_CAN = false;} return currentConvertedValue;} //reads and clears new value bool
    float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newConversionCheck = false;} return currentConvertedValue;} //reads and clears new value bool
    
    bool getNewSensorConversionCheck(){return newConversionCheck;}
    void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

};

////////////////////////////////////////////////////////////////////////////////
// Exponential Moving Average class for filtering ADC outputs.
// - Joe, 2023 April 1
class EMA
{
protected:
    bool EMA_Enable_Default = true;  //needs a set function still
    bool EMA_Enable;  //needs a set function still
    float priorEMAOutput = 0;
    float alphaEMA_Default = 0.7; //1 is no weight to old values, 0 has no weight to new value and will brick
    float alphaEMA;
    float newEMAOutput = 0;

public: 
    void setAlphaEMA(float alphaEMAIn){if(alphaEMAIn >0 && alphaEMAIn <=1){alphaEMA = alphaEMAIn;}}
    float getEMAConvertedValue(){return newEMAOutput;}
    void exponentialMovingAverage(float EMA_Input); // Always LinearMap.CurrentConvertedValue. Physically seperated to allow for easier encapsulation.
};

////////////////////////////////////////////////////////////////////////////////
// Linear Regression class for filtering ADC outputs.
//     I'm not actually sure what this class does? It doesn't look like any 
// linear regression I've seen before.  Regardless, It's here for feature parity
// during this refactor.  It also doesn't appear to have been completed.
// - Joe, 2023 April 3
class LinearRegression
{
protected:
    bool enableLinearRegressionCalc = true; //not currently using, linreg only calculates when get func requests it
    float currentLinReg_a1 = 0; // I'm pretty sure this is the slope after the regression.  I haven't checked, but that's how its used below. - Joe, 2023 April 3
    const uint32_t regressionSamples_Default = 5;
    uint32_t regressionSamples;

    // Holds an array of values used for the regression.
    // Use ALARAUtilityFunctions.h function writeToRollingArray to fill with data. - Joe 2023 April 3
    float convertedValueArray[5+3] = {};  //should be the same size as regression samples +3 for rolling array index stuff
    float timeStep = 0.01; //timeStep in seconds - Should this always have this timestep? - Joe 2023 April 3

public:
    void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment
    float linearRegressionLeastSquared_PID();
    //void setRegressionSamples():???
    //float getLinRegSlope(); // The instance below was found in EXTSensorClass.h.  I'm not sure why it has a node ID check, so I'm leaving it out for now. - Joe, 2023 April 3
    //float getLinRegSlope(){if(nodeIDCheck){currentLinReg_a1 = linearRegressionLeastSquared_PID();} return currentLinReg_a1;} 
    float getLinRegSlope(){currentLinReg_a1 = linearRegressionLeastSquared_PID(); return currentLinReg_a1;}
    bool getEnableLinearRegressionCalc(){return enableLinearRegressionCalc;}

};

////////////////////////////////////////////////////////////////////////////////
// Integral error class.
//     I'm not actually sure what this class does? It looks just like an 
// integral error term of a PID controller, but without the controller.
// - Joe, 2023 April 3
class IntegralError
{
protected:
    float maxIntegralSum_Default;
    float minIntegralSum_Default;
    float maxIntegralSum;
    float minIntegralSum;

    bool enableIntegralCalc = false;
    float currentIntegralSum = 0;

    float targetValue = 0;

public:

    bool getEnableIntegralCalc(){return enableIntegralCalc;}
    void setEnableIntegralCalc(bool setEnableIn){enableIntegralCalc = setEnableIn;}

    float getMinIntegralSum(){return minIntegralSum;}
    void setMinIntegralSum(float minIntegralSumIn){maxIntegralSum = minIntegralSumIn;}

    float getMaxIntegralSum(){return maxIntegralSum;}
    void setMaxIntegralSum(float maxIntegralSumIn){maxIntegralSum = maxIntegralSumIn;}

    void setTargetValue(float targetValueIn){targetValue = targetValueIn;}
    float getIntegralSum(){return currentIntegralSum;}

    void resetIntegralCalc(bool resetBoolIn, float integralCalcIn = 0){if(resetBoolIn){currentIntegralSum = integralCalcIn;}}  //resets the integral sum, default arg zeros it

    // Updates currentIntegralSum with a time, the current value, and the prior value.
    void accumulatedI(float timeStepAccumI, float currentValue, float priorValue);
};

////////////////////////////////////////////////////////////////////////////////
// Sample Rate defaults struct
// This is seperate to allow for convenient inits using a constructor.
// - Joe, 2023 April 6

struct sampleRateDefaults
{
    const uint32_t sampleRateSlowMode_Default;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateMedMode_Default;         //the sample rate this given sensor will be read at
    const uint32_t sampleRateFastMode_Default;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateCalibrationMode_Default; //the sample rate this given sensor will be read at

    // Default constructor.  Must supply every field.
    sampleRateDefaults(uint32_t slowMode, uint32_t medMode, uint32_t fastMode, uint32_t calibrationMode)
    : sampleRateSlowMode_Default{slowMode}, sampleRateMedMode_Default{medMode},
      sampleRateFastMode_Default{fastMode}, sampleRateCalibrationMode_Default{calibrationMode}
    {}

    // Copy constructor.
    sampleRateDefaults(const sampleRateDefaults &defs)
    : sampleRateSlowMode_Default{defs.sampleRateSlowMode_Default},
      sampleRateMedMode_Default{defs.sampleRateMedMode_Default},
      sampleRateFastMode_Default{defs.sampleRateFastMode_Default},
      sampleRateCalibrationMode_Default{defs.sampleRateCalibrationMode_Default}
    {}
};

////////////////////////////////////////////////////////////////////////////////
// Base sensor class to hold the various derived classes.
//     Currently, this class is the common elements of
// EXTDigitalDiffLCSensorClass, ALARAVRailSensorClass, ALARAHPSensorClass, and
// EXTSensorClass.  The items in private have been more thoroughly refactored
// than the items in protected.  Ideally, everything should have getters and 
// setters defined in a seperate .cpp, but the linker is whining when I try.
// - Joe
class Sensor : public Timer, public LinearMap, public EMA, public LinearRegression, public IntegralError, protected sampleRateDefaults
{

private:
    // Constant sensor values.  Usually hardware identifiers.
    const uint32_t _sensorID;
    const uint32_t _sensorNodeID;                      // NodeID the sensor is controlled by
    const ADCType _sensorSource = ADCType::TeensyMCUADC;  //default source here is Teensy ADC
    const uint8_t _ADCinput; //the input that will be read for this sensor that will get used in the ADC read main loop

protected:
    
    SensorState sensorState;

    uint32_t sampleRateSlowMode;        //the sample rate this given sensor will be read at
    uint32_t sampleRateMedMode;         //the sample rate this given sensor will be read at
    uint32_t sampleRateFastMode;        //the sample rate this given sensor will be read at
    uint32_t sampleRateCalibrationMode; //the sample rate this given sensor will be read at

// const uint32_t sampleRateSlowMode_Default;        //the sample rate this given sensor will be read at
// const uint32_t sampleRateMedMode_Default;         //the sample rate this given sensor will be read at
// const uint32_t sampleRateFastMode_Default;        //the sample rate this given sensor will be read at
// const uint32_t sampleRateCalibrationMode_Default; //the sample rate this given sensor will be read at

    uint32_t _currentSampleRate = 10;         // Sample rates are in samples per second
    uint32_t currentRawValue{};               // holds the current value for the sensor

    //bool pullTimestamp = false; // This is in public, for some reason.

    bool nodeIDCheck = false;                           // Whether this object should operate on this node
    bool internalMCUTemp;                       // Is this sensor the MCU internal temp
    bool newSensorValueCheck_CAN = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newSensorConvertedValueCheck_CAN = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newSensorValueCheck_Log = false;                      // Is the current raw value a new read that hasn't been sent yet?

    uint16_t currentCANtimestamp = 0;
    uint32_t currentTimestampSeconds = 0;
    uint32_t currentTimestampMicros = 0;
    uint32_t priorTimestampSeconds = 0;
    uint32_t priorTimestampMicros = 0;

    // Linear regression Calibration group
    //uint16_t rollingSensorArrayRaw[10];       // Array for doing averages of readings
    //uint8_t currentRollingArrayPosition = 0;
    uint32_t currentCalibrationValue{};               // holds the current value for the sensor
    //uint32_t currentRunningSUM = 0;

public:
    //SENSORBASE(){};
    Sensor(uint32_t sensorID,       uint32_t sensorNodeID,    uint8_t ADCinput, const sampleRateDefaults&SRD) // The lack of a space here feels weird, but it compiles! - Joe, 2023 April 6
        : _sensorID{sensorID}, _sensorNodeID{sensorNodeID}, _ADCinput{ADCinput},      sampleRateDefaults{SRD},
        LinearMap{} {};

    Sensor(uint32_t sensorID,       uint32_t sensorNodeID,    uint8_t ADCinput, const sampleRateDefaults&SRD,        ADCType sensorSource)
        : _sensorID{sensorID}, _sensorNodeID{sensorNodeID}, _ADCinput{ADCinput},      sampleRateDefaults{SRD}, _sensorSource{sensorSource},
        LinearMap{} {};


    bool pullTimestamp = false;
    virtual void begin() = 0;                     //
    virtual void resetAll() = 0;
    virtual void read(ADC& adc) = 0;              // updates currentRawValue with current reading, using an activated ADC object
    void stateOperations(); // No longer virtual!

    // LinearMap group
    // Maps ADC read value to the calibrated range using linConvCoef.
    // Accepts currentRawValue, or currentRawDiffValue.
// void linearConversion(uint32_t); 
// void linearConversion_WithSecondary(uint32_t); // Maps ADC read value to the calibrated range using linConvCoef, using both calibration steps.
// 
// float getCurrentConvertedValue(){return currentConvertedValue;}
// //float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newSensorConvertedValueCheck_CAN = false;} return currentConvertedValue;} //reads and clears new value bool
// float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newConversionCheck = false;} return currentConvertedValue;} //reads and clears new value bool
// 
// bool getNewSensorConversionCheck(){return newConversionCheck;}
// void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

    // EMA group
// void setAlphaEMA(float alphaEMAIn){if(alphaEMAIn >0 && alphaEMAIn <=1){alphaEMA = alphaEMAIn;}}
// float getEMAConvertedValue(){return newEMAOutput;}
// void exponentialMovingAverage(float EMA_Input); // Always CurrentConvertedValue. Physically seperated to allow for easier encapsulation.

    // Linear Regression group
// void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment
// float linearRegressionLeastSquared_PID();
// //void setRegressionSamples():???
// virtual float getLinRegSlope() = 0;
// bool getEnableLinearRegressionCalc(){return enableLinearRegressionCalc;}

    // Integral error group
// bool getEnableIntegralCalc(){return enableIntegralCalc;}
// 
// float getMaxIntegralSum(){return maxIntegralSum;}
// float getMinIntegralSum(){return minIntegralSum;}
// 
// void setMaxIntegralSum(float maxIntegralSumIn){maxIntegralSum = maxIntegralSumIn;}
// void setMinIntegralSum(float minIntegralSumIn){maxIntegralSum = minIntegralSumIn;}
// 
// void setTargetValue(float targetValueIn){targetValue = targetValueIn;}
// float getIntegralSum(){return currentIntegralSum;}
// 
// void setEnableIntegralCalc(bool setEnableIn){enableIntegralCalc = setEnableIn;}
// void resetIntegralCalc(bool resetBoolIn, float integralCalcIn = 0){if(resetBoolIn){currentIntegralSum = integralCalcIn;}}  //resets the integral sum, default arg zeros it

    // Semi-compositional implementation. Calls accumulateI from IntegralError with appropriate inputs.
    // Updates currentIntegralSum from IntegralError using currentConvertedValue and priorConvertedValue from LinearMap.
    void accumulatedI_float(); 

    // Access functions defined in place
    uint32_t getSensorID() const {return _sensorID;}
    uint32_t getSensorNodeID() const {return _sensorNodeID;}

    ADCType getADCtype() const {return _sensorSource;}
    uint32_t getADCinput(){return _ADCinput;};
    //virtual uint32_t getADCinput(bool input1);

    uint32_t getCurrentSampleRate(){return _currentSampleRate;}
    uint32_t getCurrentRawValue(){return currentRawValue;}
    uint32_t getCurrentRawValue(bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} return currentRawValue;} //reads and clears new value bool

    uint16_t getCANTimestamp(){return currentCANtimestamp;}
    uint32_t getTimestampSeconds(){return currentTimestampSeconds;}
    uint32_t getTimestampMicros(){return currentTimestampMicros;}
    //virtual uint8_t getCurrentRollingArrayPosition(){return currentRollingArrayPosition;}
    uint32_t getCurrentRollingAverage(){return currentCalibrationValue;}

    bool getNodeIDCheck(){return nodeIDCheck;}
    bool getNewSensorValueCheckCAN(){return newSensorValueCheck_CAN;}
    bool getNewSensorValueCheckLog(){return newSensorValueCheck_Log;}

    void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;} // set the Node ID Check bool function
    void setState(SensorState newState){sensorState = newState;}
    void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros)
    {
        if (pullTimestamp)
        {
        priorTimestampSeconds = currentTimestampSeconds;  //shifts the previous current into prior variables
        priorTimestampMicros = currentTimestampMicros;
        currentTimestampSeconds = timestampSeconds;       //sets the new current timestamps from input arguments
        currentTimestampMicros = timestampMicros;
        pullTimestamp = false;
        }
    }

    //virtual float getDeengergizeOffsetValue();
    

//virtual void initializeLinReg(uint8_t arraySizeIn) = 0; //not in use at the moment

    void setCurrentRawValue(uint32_t updateCurrentRawValue){currentRawValue = updateCurrentRawValue;} //Why is set raw value here again?
    //resets both the CAN and log bools for when new sample is read
    void setNewSensorValueCheck(bool updateNewSensorValueCheck){newSensorValueCheck_CAN = updateNewSensorValueCheck; newSensorValueCheck_Log = updateNewSensorValueCheck;} 
    void setCANTimestamp(uint16_t CANTimestamp){currentCANtimestamp = CANTimestamp;}

    //void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {_currentSampleRate = updateCurrentSampleRate; newSensorValueCheck = true; newConversionCheck = false;}
    void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {_currentSampleRate = updateCurrentSampleRate;}
    //virtual void setCurrentSampleRate(uint32_t updateCurrentSampleRate);
    
    void setSampleRateSlowMode(uint32_t updateSampleRateSlowMode) {if(updateSampleRateSlowMode<=2000){sampleRateSlowMode = updateSampleRateSlowMode;}}
    void setSampleRateMedMode(uint32_t updateSampleRateMedMode)   {if(updateSampleRateMedMode<=2000){sampleRateMedMode = updateSampleRateMedMode;}}
    void setSampleRateFastMode(uint32_t updateSampleRateFastMode) {if(updateSampleRateFastMode<=2000){sampleRateFastMode = updateSampleRateFastMode;}}

//void setTargetValue(float targetValueIn){targetValue = targetValueIn;}
    //virtual void setDeenergizeOffset(ADC& adc, bool outputOverrideIn);

    // This group feels like it should be a different class, or object.
//void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment
//float linearRegressionLeastSquared_PID();
//void accumulatedI_float();

    /*     //void setRollingSensorArrayRaw(uint8_t arrayPosition, uint16_t sensorValueToArray)
    void setRollingSensorArrayRaw(uint8_t arrayPosition, uint16_t sensorValueToArray)
      {
        rollingSensorArrayRaw[arrayPosition] = sensorValueToArray;
        arrayPosition++;
      } */

/*     void setCurrentCalibrationValue()
    {
    for (size_t i = 0; i < 10; i++)
    {
      currentRunningSUM = currentRunningSUM + rollingSensorArrayRaw[i];
    }
    currentCalibrationValue = currentRunningSUM / 10;
    } */
    //};
};

// need to add differential read toggle somehow 
// - differential boolean variable that allows second input to be chosen or defaulted to correct option
// need to add a way to set other SENSOR types like the RTD sensors over I2C (we'd probably want multiple classes. ADCsensors, I2C sensors, SPI sensors etc - Mat)
// - maybe not the right call to roll into this? Hmm. Need to establish use of SENSOR class with sample rates and real read/sends to see what is better
// That will set me up for incorporating the external ADCs later

#endif