
#ifndef BASECLASS_SENSORCLASS_H_
#define BASECLASS_SENSORCLASS_H_

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "./States/SensorStates.hpp"
#include "Timer.hpp"
//#include "fluidSystemSimulation.h"
#include "./ALARAUtilityFunctions.h"
//#pragma once
//#pragma GCC diagnostic ignored "-Wreorder" // go away cringe
#include <limits> // For IntegralError
#include "./Base_Classes/ID.hpp"
#include "./Base_Classes/Task_Begin.hpp"

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
private:
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
    // Default constructor.
    // Initializes the LinearMap with no mapping.
    LinearMap():
        linConvCoef1_m_Default{0}, linConvCoef1_b_Default{1},
        linConvCoef1_m{0},         linConvCoef1_b{1},
        linConvCoef2_m_Default{0}, linConvCoef2_b_Default{1},
        linConvCoef2_m{0},         linConvCoef2_b{1}
        {}
    
     // Initializes the LinearMap with no mapping on the secondary.
     LinearMap(float setLinConvCoef1_m_Default, float setLinConvCoef1_b_Default):
         linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default},
         linConvCoef1_m{        setLinConvCoef1_m_Default}, linConvCoef1_b{        setLinConvCoef1_b_Default},
         linConvCoef2_m_Default{0},linConvCoef2_b_Default{1},
         linConvCoef2_m{0},        linConvCoef2_b{1}
         {}
     
     // Initializes the LinearMap with both mappings.
     LinearMap(float setLinConvCoef1_m_Default, float setLinConvCoef1_b_Default, float setLinConvCoef2_m_Default, float setLinConvCoef2_b_Default):
         linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default},
         linConvCoef1_m{        setLinConvCoef1_m_Default}, linConvCoef1_b{        setLinConvCoef1_b_Default},
         linConvCoef2_m_Default{setLinConvCoef2_m_Default}, linConvCoef2_b_Default{setLinConvCoef2_b_Default},
         linConvCoef2_m{        setLinConvCoef2_m_Default}, linConvCoef2_b{        setLinConvCoef2_b_Default}
         {}
     
     // Copy constructor. 
     LinearMap(const LinearMap &Other):
         linConvCoef1_m_Default{Other.linConvCoef1_m_Default}, linConvCoef1_b_Default{Other.linConvCoef1_b_Default},
         linConvCoef1_m{        Other.linConvCoef1_m_Default}, linConvCoef1_b{        Other.linConvCoef1_b_Default},
         linConvCoef2_m_Default{Other.linConvCoef2_m_Default}, linConvCoef2_b_Default{Other.linConvCoef2_b_Default},
         linConvCoef2_m{        Other.linConvCoef2_m_Default}, linConvCoef2_b{        Other.linConvCoef2_b_Default}
         {}

    // Maps ADC read value to the calibrated range using linConvCoef.
    // Accepts currentRawValue, or currentRawDiffValue.
    void linearConversion(uint32_t currentRaw); 
    void linearConversion_WithSecondary(uint32_t currentRaw); // Maps ADC read value to the calibrated range using linConvCoef, using both calibration steps.

    float getCurrentConvertedValue(){return currentConvertedValue;}
    //float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newSensorConvertedValueCheck_CAN = false;} return currentConvertedValue;} //reads and clears new value bool
    float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newConversionCheck = false;} return currentConvertedValue;} //reads and clears new value bool

    bool getNewSensorConversionCheck(){return newConversionCheck;}
    void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

    void resetAll()
    {
        linConvCoef1_m = linConvCoef1_m_Default;
        linConvCoef1_b = linConvCoef1_b_Default;
        linConvCoef2_m = linConvCoef2_m_Default;
        linConvCoef2_b = linConvCoef2_b_Default;
    }

    // I dont like these being here, but its needed to fully isolate this. - Joe, 2023 October 5
    float getPriorConvertedValue(){return priorConvertedValue;}

    // Please do not use this function.  I dont think anything should need to override these values. - Joe, 2023 October 5
    void _overrideValues(float prior, float current)
    {
        priorConvertedValue = prior;
        currentConvertedValue = current;
    }

};

////////////////////////////////////////////////////////////////////////////////
// Exponential Moving Average class for filtering ADC outputs.
// - Joe, 2023 April 1
class EMA
{
private:
    bool _EMA_Enable_Default = true;  //needs a set function still
    bool _EMA_Enable;  //needs a set function still
    float _priorEMAOutput = 0;
    float _alphaEMA_Default = 0.7; //1 is no weight to old values, 0 has no weight to new value and will brick
    float _alphaEMA;
    float _newEMAOutput = 0;

    ////////////////////////////////////////////////////////////////////////////////
    //     The comment on _alphaEMA_Default scares me.  This function is an 
    // initialization guard that only should be used in the initializer 
    // constructor. - Joe, 2023 October 5
    // 
    // Limits values to be between 0 and 1, excluding 0.  If less than 0, sets to 0.7.  Clamps 1.
    static float Initializer_Guard(float alphaEMAIn)
    {
        if(alphaEMAIn <= 0) return 0.7; // Magic number.  This is an arbitrary value.  I can't pick one that's epsilon away - Joe, 2023 October 5
        if(1 < alphaEMAIn) return 1;
        return alphaEMAIn;
    }

public:
    // Default constructor.
    EMA():
        _alphaEMA_Default{0.7}, _EMA_Enable_Default{true},
        _alphaEMA{0.7},         _EMA_Enable{true}
    {}

    // Copy constructor.
    EMA(const EMA &Other):
        _alphaEMA_Default{Other._alphaEMA_Default}, _EMA_Enable_Default{Other._EMA_Enable_Default},
        _alphaEMA{        Other._alphaEMA},         _EMA_Enable{        Other._EMA_Enable}
    {}

    // Initializer constructor.
    // set_alphaEMA_Default has a guard on its initialization value.
    EMA(float set_alphaEMA_Default, bool set_EMA_Enable_Default):
        _alphaEMA_Default{Initializer_Guard(set_alphaEMA_Default)}, _EMA_Enable_Default{set_EMA_Enable_Default},
        _alphaEMA{        Initializer_Guard(set_alphaEMA_Default)}, _EMA_Enable{        set_EMA_Enable_Default}
    {}

    // Sets the EMA alpha.  Limits the values to be between 0 and 1, excluding 0.
    void setAlphaEMA(float alphaEMAIn){if(alphaEMAIn >0 && alphaEMAIn <=1){_alphaEMA = alphaEMAIn;}}
    float getEMAConvertedValue(){return _newEMAOutput;}
    void exponentialMovingAverage(float EMA_Input); // Always LinearMap.CurrentConvertedValue. Physically seperated to allow for easier encapsulation.

    // Resets all settings with a default.
    void resetAll()
    {
        _alphaEMA = _alphaEMA_Default;
        _EMA_Enable = _EMA_Enable_Default;
    }
};

////////////////////////////////////////////////////////////////////////////////
// Linear Regression class for filtering ADC outputs.
//     I'm not actually sure what this class does? It doesn't look like any 
// linear regression I've seen before.  Regardless, It's here for feature parity
// during this refactor.  It also doesn't appear to have been completed.
//     Dan Morgan said that this is PID D term filtering! Used for the old Bang 
// Bang controller. - Joe, 2023 April 6
class LinearRegression
{
private:
    bool _enableLinearRegressionCalc_Default = true; //not currently using, linreg only calculates when get func requests it
    bool _enableLinearRegressionCalc = true; //not currently using, linreg only calculates when get func requests it

    // Do not let the user modify this without writing a destructor. - Joe Kessler, 2023 October 8
    const static uint32_t _regressionSamples_Default = 5;
    uint32_t _regressionSamples;

    const float _timeStep_Default = 0.01;
    float _timeStep = 0.01; //timeStep in seconds - Should this always have this timestep? - Joe 2023 April 3

    // Holds an array of values used for the regression.
    // Use ALARAUtilityFunctions.h function writeToRollingArray to fill with data. - Joe 2023 April 3
    int _convertedValueArrayNextInd;
    float _convertedValueArray[_regressionSamples_Default];  //should be the same size as regression samples +3 for rolling array index stuff
    
    float _currentLinReg_a1 = 0; // I'm pretty sure this is the slope after the regression.  I haven't checked, but that's how its used below. - Joe, 2023 April 3

public:
    // Default constructor.
    LinearRegression():
        _enableLinearRegressionCalc_Default{true}, //_regressionSamples_Default{__regressionSamples_SIZE},
        _enableLinearRegressionCalc{        true}, _regressionSamples{_regressionSamples_Default},
        _timeStep_Default{0.01}, _currentLinReg_a1{0},
        _timeStep{        0.01},
        _convertedValueArrayNextInd{0},
        //     This initializer is "clever," but I can't find a cleaner way that
        // works.  An empty curly brace sets all the values to 0.
        // - Joe 2023 October 8
        _convertedValueArray{{}}
    {}

    // Copy constructor.
    LinearRegression(const LinearRegression &Other):
        _enableLinearRegressionCalc_Default{Other._enableLinearRegressionCalc_Default},
        _enableLinearRegressionCalc{        Other._enableLinearRegressionCalc},
        //_regressionSamples_Default{Other._regressionSamples_Default},
        _regressionSamples{        Other._regressionSamples},
        _timeStep_Default{Other._timeStep_Default},
        _timeStep{        Other._timeStep},
        _convertedValueArrayNextInd{Other._convertedValueArrayNextInd},
        _currentLinReg_a1{Other._currentLinReg_a1}
    {
        memcpy(_convertedValueArray, Other._convertedValueArray, _regressionSamples*sizeof(float));
    }

    // Initializer constructor.
    // regressionSamples_Default currently cannot be changed.
    //LinearRegression(bool enableLinearRegressionCalc_Default, float timeStep_Default):


    ////////////////////////////////////////////////////////////////////////////
    //     This class should have more parameters and a parameter constructor,
    // but it currently doesn't have any tunable settings.  I'm only 
    // implementing this default constructor for consistency across the other 
    // things Sensor uses, until it has a use in the future.  It's currently 
    // broken and unused, so please fix it if you intend to use it. Or bother
    // me to do it! - Joe Kessler, 2023 October 8

    void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment
    float linearRegressionLeastSquared_PID();
    //void setRegressionSamples():???
    //float getLinRegSlope(); // The instance below was found in EXTSensorClass.h.  I'm not sure why it has a node ID check, so I'm leaving it out for now. - Joe, 2023 April 3
    //float getLinRegSlope(){if(nodeIDCheck){currentLinReg_a1 = linearRegressionLeastSquared_PID();} return currentLinReg_a1;} 
    float getLinRegSlope(){_currentLinReg_a1 = linearRegressionLeastSquared_PID(); return _currentLinReg_a1;}
    bool getEnableLinearRegressionCalc(){return _enableLinearRegressionCalc;}

    float* getConvertedValueArrayPtr() {return _convertedValueArray;}

    // Initializes the first 3 elements of _convertedValueArray.
    // A vectorized form of this might make initializeLinReg obsolete. - Joe, 2023 October 8
    void initConvertedValueArray(float a0, float a1, float a2)
    {
        _convertedValueArray[0] = a0;
        _convertedValueArray[1] = a1;
        _convertedValueArray[2] = a2;
    }

    void initConvertedValueArray(int n, float a)
    {
        _convertedValueArray[n] = a;
    }

    void writeToRollingArray(float newInputArrayValue)
    {
        _convertedValueArrayNextInd = (_convertedValueArrayNextInd+1) % _regressionSamples;
        _convertedValueArray[_convertedValueArrayNextInd] = newInputArrayValue;
    }

    uint32_t getRegressionSamples(){return _regressionSamples;}

    float getTimeStep(){return _timeStep;}
    void setTimeStep(float timeStep){_timeStep = timeStep;}

    void resetAll()
    {
        _enableLinearRegressionCalc = _enableLinearRegressionCalc_Default;
        _regressionSamples = _regressionSamples_Default; // Should never change, but lets use it anyway. - Joe Kessler, 2023 October 8
        _timeStep = _timeStep_Default;
    }

};

////////////////////////////////////////////////////////////////////////////////
// Integral error class.
//     I'm not actually sure what this class does? It looks just like an 
// integral error term of a PID controller, but without the controller.
// - Joe, 2023 April 3
//     Dan Morgan said that this is PID I term filtering! Used for the old Bang 
// Bang controller. - Joe, 2023 April 6
class IntegralError
{
private:
    float _maxIntegralSum_Default;
    float _minIntegralSum_Default;
    float _maxIntegralSum;
    float _minIntegralSum;

    bool _enableIntegralCalc = false;
    float _currentIntegralSum = 0;

    float _targetValue = 0;

public:
    // Default constructor.
    // This allows the code to be shared, but disables the clipping check.
    // std::numeric_limits<float>::max(); // Max value of float
    // std::numeric_limits<float>::lowest(); // Max negative value of float
    // This is kinda long, I need to find a way to condense lowest and max to be easier to read. - Joe Kessler, 2023 October 8
    IntegralError():
        _minIntegralSum_Default{std::numeric_limits<float>::lowest()}, _maxIntegralSum_Default{std::numeric_limits<float>::max()}, 
        _minIntegralSum{        std::numeric_limits<float>::lowest()}, _maxIntegralSum{        std::numeric_limits<float>::max()}, 
        _enableIntegralCalc{false}
    {}

    // Initializer constructor.
    IntegralError(        float minIntegralSum_Default, float maxIntegralSum_Default, bool enableIntegralCalc):
        _minIntegralSum_Default{minIntegralSum_Default},     _maxIntegralSum_Default{maxIntegralSum_Default}, 
        _minIntegralSum{        minIntegralSum_Default},     _maxIntegralSum{        maxIntegralSum_Default}, 
        _enableIntegralCalc{enableIntegralCalc}
    {}

    // Copy constructor.
    IntegralError(const IntegralError &Other):
        _minIntegralSum_Default{Other._minIntegralSum_Default}, _maxIntegralSum_Default{Other._maxIntegralSum_Default},
        _minIntegralSum{        Other._minIntegralSum},         _maxIntegralSum{        Other._maxIntegralSum},
        _enableIntegralCalc{Other._enableIntegralCalc}
    {}

    bool getEnableIntegralCalc(){return _enableIntegralCalc;}
    void setEnableIntegralCalc(bool setEnableIn){_enableIntegralCalc = setEnableIn;}

    float getMinIntegralSum(){return _minIntegralSum;}
    void setMinIntegralSum(float minIntegralSumIn){_minIntegralSum = minIntegralSumIn;}

    float getMaxIntegralSum(){return _maxIntegralSum;}
    void setMaxIntegralSum(float maxIntegralSumIn){_maxIntegralSum = maxIntegralSumIn;}

    void setTargetValue(float targetValueIn){_targetValue = targetValueIn;}
    float getIntegralSum(){return _currentIntegralSum;}

    // Updates currentIntegralSum with a time, the current value, and the prior value.
    void accumulatedI(float timeStepAccumI, float currentValue, float priorValue);

    //resets the integral sum, default arg zeros it 
    void resetIntegralCalc(bool resetBoolIn, float integralCalcIn = 0)
    {
        if(resetBoolIn)
        {
            _currentIntegralSum = integralCalcIn;
        }
    }  

    void resetAll()
    {
        resetIntegralCalc(true);
        _minIntegralSum = _minIntegralSum_Default;
        _maxIntegralSum = _maxIntegralSum_Default;
    }
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

    // Initializer constructor.  Must supply every field.
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
class Sensor : protected sampleRateDefaults, public Task_Begin
{

private:
    // Constant sensor values.  Usually hardware identifiers.
//const uint32_t _sensorID;
//const uint32_t _sensorNodeID;                      // NodeID the sensor is controlled by
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

// bool nodeIDCheck = false;                           // Whether this object should operate on this node
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
    Sensor(const idClass&sensorID,    uint8_t ADCinput, const sampleRateDefaults&SRD,
        const LinearMap&linearMap, const EMA&ema, const LinearRegression&linearReg, const IntegralError&IErr) // The lack of a space here feels weird, but it compiles! - Joe, 2023 April 6
        :             ID{sensorID}, _ADCinput{ADCinput},      sampleRateDefaults{SRD},
            __linearMap{linearMap},    __ema{ema},           __linearReg{linearReg},             __IErr{IErr}
    {}

    // Initializer with sensorSource.
    Sensor(const idClass&sensorID,    uint8_t ADCinput, const sampleRateDefaults&SRD,  
        const LinearMap&linearMap, const EMA&ema, const LinearRegression&linearReg, const IntegralError&IErr,
              ADCType sensorSource)
        :             ID{sensorID}, _ADCinput{ADCinput},      sampleRateDefaults{SRD},
            __linearMap{linearMap},    __ema{ema},           __linearReg{linearReg},             __IErr{IErr},
        _sensorSource{sensorSource}
    {};

    bool pullTimestamp = false;
    //virtual void begin() = 0; // Does this pass the pure virtual function down the chain? I guess it does! - Joe 2023 Oct 11
    virtual void resetAll() = 0;
    virtual void readRaw(ADC& adc);           // updates currentRawValue with current reading, using an activated ADC object
    virtual void readSim(ADC& adc){};         // updates currentConvertedValue with a simulated reading.  Default does nothing.
    virtual void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    void stateOperations(); // No longer virtual!

    // TODO: refactor (automatic?) to remove one underscore from each of these member classes.
    idClass ID;
    Timer __timer;
    LinearMap __linearMap;
    EMA __ema;
    LinearRegression __linearReg;
    IntegralError __IErr;

    // Resets all component classes that each have a resetAll function.
    void resetAllComponents()
    {
        __linearMap.resetAll();
        __ema.resetAll();
        __linearReg.resetAll();
        __IErr.resetAll();
    }

    // Semi-compositional implementation. Calls accumulateI from IntegralError with appropriate inputs.
    // Updates currentIntegralSum from IntegralError using currentConvertedValue and priorConvertedValue from LinearMap.
    void accumulatedI_float(); 

    ADCType getADCtype() const {return _sensorSource;}
    uint32_t getADCinput(){return _ADCinput;};
    //virtual uint32_t getADCinput(bool input1);

    uint32_t getCurrentSampleRate(){return _currentSampleRate;}
    uint32_t getCurrentRawValue(){return currentRawValue;}
    uint32_t getCurrentRawValue(bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} return currentRawValue;} //reads and clears new value bool

    uint16_t getCANTimestamp(){return currentCANtimestamp;}
    uint32_t getTimestampSeconds(){return currentTimestampSeconds;}
    uint32_t getTimestampMicros(){return currentTimestampMicros;}

    uint32_t getCurrentRollingAverage(){return currentCalibrationValue;}

    bool getNewSensorValueCheckCAN(){return newSensorValueCheck_CAN;}
    bool getNewSensorValueCheckLog(){return newSensorValueCheck_Log;}

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

};

// need to add differential read toggle somehow 
// - differential boolean variable that allows second input to be chosen or defaulted to correct option
// need to add a way to set other SENSOR types like the RTD sensors over I2C (we'd probably want multiple classes. ADCsensors, I2C sensors, SPI sensors etc - Mat)
// - maybe not the right call to roll into this? Hmm. Need to establish use of SENSOR class with sample rates and real read/sends to see what is better
// That will set me up for incorporating the external ADCs later

#endif