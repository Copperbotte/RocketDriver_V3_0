
#ifndef BASECLASS_TIMERCLASS_H_
#define BASECLASS_TIMERCLASS_H_

#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////
// Base timer class to easily inherit the timer and common functions.
//     This structure is used all over the code.  Putting it here as a common 
// element I feel obfuscates some of the code, but it allows for an smoother
// transition to composition-based programming from polymorphism.
// 
// This class has proper usage of const.
// - Joe, 2023 March 31


class Timer
{
    // timer for the valve, used for changing duty cycles, in MICROS
    elapsedMicros timer = 0; // Initializes new timers to 0.

public:
    const elapsedMicros getTimer(){return timer;}
    void resetTimer(){timer = 0;} // resets timer to zero, timer increments automatically in microseconds
};

#endif