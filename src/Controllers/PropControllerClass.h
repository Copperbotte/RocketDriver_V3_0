#ifndef PROPCONTROLLERCLASS_H
#define PROPCONTROLLERCLASS_H

#include <Arduino.h>
#include "./States/ControllerStates.h"
#include "./Base_Classes/ID.hpp"
#include "./Base_Classes/Task_Begin.hpp"
#include "./Base_Classes/Timer.hpp"

#define PROPCONTROLLERID 0

class PropulsionController : public Timer, public Task_Begin
{
private:
    ControllerState state;
    ControllerState priorState;

    uint32_t targetValue;
    uint32_t deadbandHighPoint;
    uint32_t deadbandLowPoint;
    uint32_t valveMinimumEnergizeTime;
    uint32_t valveMinimumDeenergizeTime;

public:

    idClass ID;

    // constructor
    PropulsionController(uint8_t setControllerNodeID, uint32_t setTargetValue, bool setNodeIDCheck = false);
    
    // a start up method, to set pins from within setup()
    void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
    uint32_t getTargetValue(){return targetValue;}

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
    void stateOperations();


};


#endif