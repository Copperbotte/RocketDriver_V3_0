#ifndef PROPCONTROLLERCLASS_H
#define PROPCONTROLLERCLASS_H

#include <Arduino.h>
#include "./States/ControllerStates.h"
#include "./Base_Classes/Task_Begin.hpp"
#include "./Base_Classes/Timer.hpp"

class PropulsionController : public Timer, public Task_Begin
{
private:
    const uint8_t controllerNodeID;
    bool nodeIDCheck;                           // Whether this object should operate on this node
    ControllerState state;
    ControllerState priorState;

    uint32_t targetValue;
    uint32_t deadbandHighPoint;
    uint32_t deadbandLowPoint;
    uint32_t valveMinimumEnergizeTime;
    uint32_t valveMinimumDeenergizeTime;

public:

    // constructor
        PropulsionController(uint8_t setControllerNodeID, uint32_t setTargetValue, bool setNodeIDCheck = false);
    // a start up method, to set pins from within setup()
    void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
        uint8_t getControllerNodeID(){return controllerNodeID;}
        bool getNodeIDCheck(){return nodeIDCheck;}
        uint32_t getTargetValue(){return targetValue;}
    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();


};


#endif