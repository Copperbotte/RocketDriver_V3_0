
#ifndef BASECLASS_CONTROLLER_H_
#define BASECLASS_CONTROLLER_H_

#include "./Base_Classes/state_machine.hpp"
#include "./States/SensorStates.hpp"


////////////////////////////////////////////////////////////////////////////////
// Base controller class to hold the various derived classes.
//     Currently, this class is the common elements of EngineController and 
// TankPressController.  This has the same pitfalls as described in Sensor.hpp.
// - Joe, 2023 July 17
template<typename StateType>
class Controller : public StateMachine_controllerConfig<StateType>
{
protected:
    // These constants are being a pain right now, handle them later. - Joe, 2023 July 17
    //     In fact, this entire file is being a pain.  Maybe I need to organize the
    // functions into groups like I did with sensor before moving them here.
    const uint32_t controllerID;                          // Controller ID number 
    const uint8_t controllerNodeID;
    bool nodeIDCheck;                           // Whether this object should operate on this node
    bool testPass = false;
// StateType state; // These two are handled by the state machine template.
// StateType priorState;
    int64_t currentAutosequenceTime;
    SensorState sensorState;                    // Use one sensor state inside here to toggle all sensors on controller

public:
////////////////////////////////////////////////////////////////////////////////
    //     I have no idea why this doesn't work in the .cpp file. It works on
    // sensors, so there's probably a way to make it work here too.
    // - Joe, 2023 September 28
    Controller(const uint32_t _controllerID,     const uint8_t _controllerNodeID,         bool setNodeIDCheck = false): // Is this an optional parameter?? I didn't know c++ had these. - Joe 2023 September 29
                 controllerID{_controllerID}, controllerNodeID{_controllerNodeID}, nodeIDCheck{setNodeIDCheck}
    {
    };

    virtual void begin() = 0; // a start up method, to set pins from within setup()

    uint32_t getControllerID(){return controllerID;}
    uint8_t getControllerNodeID(){return controllerNodeID;}
    bool getNodeIDCheck(){return nodeIDCheck;}

    SensorState getControllerSensorState(){return sensorState;}

};



#endif