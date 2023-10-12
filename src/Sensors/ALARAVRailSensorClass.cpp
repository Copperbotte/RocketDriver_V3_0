#include "ALARAVRailSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>
#include <limits>

// Initializer 1
/*
// Why is this disabled? I'm not updating it to the new format if it's gonna get deleted anyway. - Joe, 2023 Oct 12
ALARAHP_SENSOR::ALARAHP_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off)
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
}
*/

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

    sensorState = setSensorState;
}


void ALARAVRAIL_SENSOR::begin()
{
    if (ID.getNodeIDCheck())
    {
        pinMode(getADCinput(), INPUT);
    }
}

void ALARAVRAIL_SENSOR::resetAll()
{
    sampleRateSlowMode = sampleRateSlowMode_Default;
    sampleRateMedMode = sampleRateMedMode_Default;
    sampleRateFastMode = sampleRateFastMode_Default;
    sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

    resetAllComponents();
}
