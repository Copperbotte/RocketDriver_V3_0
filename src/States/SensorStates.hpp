#ifndef STATES_SENSORSTATES_H_
#define STATES_SENSORSTATES_H_
#pragma once

// Joe - 2023 April 15
// https://stackoverflow.com/questions/261963/how-can-i-iterate-over-an-enum

enum SensorState
{
    Off,
    Slow,
    Medium,
    Fast,
    Calibration, //doesn't currently work - new partially implemented feature
};
/*
#define SWITCH_KEY(Key) case Key: return #Key
const char* SensorState_Key(const SensorState State)
{
    switch(State)
    {
    SWITCH_KEY(SensorState::Off);
    SWITCH_KEY(SensorState::Slow);
    SWITCH_KEY(SensorState::Medium);
    SWITCH_KEY(SensorState::Fast);
    SWITCH_KEY(SensorState::Calibration);
    };
    return "[KEY ERROR]";
}
#undef SWITCH_KEY
*/
#endif