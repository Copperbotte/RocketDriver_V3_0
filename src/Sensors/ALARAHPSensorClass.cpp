#include "ALARAHPSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>


// Initializer 1
ALARAHP_SENSOR::ALARAHP_SENSOR(const idClass&setSensorID, uint8_t setADCinput, const LinearMap&linearMap, uint32_t setCurrentSampleRate, SensorState setSensorState)
    : Sensor{setSensorID, setADCinput, _SRD(), linearMap, EMA{}, LinearRegression{}, IntegralError{}}
{
    // setting stuff to defaults at initialization
    sampleRateSlowMode = sampleRateSlowMode_Default;
    sampleRateMedMode = sampleRateMedMode_Default;
    sampleRateFastMode = sampleRateFastMode_Default;
    sampleRateCalibrationMode = sampleRateCalibrationMode_Default;
    _currentSampleRate = setCurrentSampleRate;

    sensorState = setSensorState;
}


void ALARAHP_SENSOR::begin()
{
    if (ID.getNodeIDCheck())
    {
        pinMode(getADCinput(), INPUT);
    }
}

void ALARAHP_SENSOR::resetAll()
{
    sampleRateSlowMode = sampleRateSlowMode_Default;
    sampleRateMedMode = sampleRateMedMode_Default;
    sampleRateFastMode = sampleRateFastMode_Default;
    sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

    resetAllComponents();
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
            //priorConvertedValue = currentConvertedValue;
            //currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;
            bool newConversionCheck_temp = __linearMap.getNewSensorConversionCheck();
            __linearMap.linearConversion(currentRawValue);
            __linearMap.setNewConversionCheck(newConversionCheck_temp);

            __ema.exponentialMovingAverage(__linearMap.getCurrentConvertedValue());
            OffsetFunctimer = 0;
        }
    // while output override still has this running, update the deenergize offset
    deenergizeOffset = __ema.getEMAConvertedValue();
    }
}
