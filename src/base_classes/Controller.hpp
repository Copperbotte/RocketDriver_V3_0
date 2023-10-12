
#ifndef BASECLASS_CONTROLLER_H_
#define BASECLASS_CONTROLLER_H_

#include "./Base_Classes/Controller.hpp"
#include "./Base_Classes/ID.hpp"
#include "./Base_Classes/Task_Begin.hpp"
#include "./Base_Classes/state_machine.hpp"
#include "./States/SensorStates.hpp"



////////////////////////////////////////////////////////////////////////////////
// Base controller class to hold the various derived classes.
//     Currently, this class is the common elements of EngineController and 
// TankPressController.  This has the same pitfalls as described in Sensor.hpp.
// - Joe, 2023 July 17
template<typename StateType>
class Controller : public StateMachine_controllerConfig<StateType>, public Task_Begin
{
protected:
    bool testPass = false;
    int64_t currentAutosequenceTime;
    SensorState sensorState;                    // Use one sensor state inside here to toggle all sensors on controller

public:
////////////////////////////////////////////////////////////////////////////////
    //     I have no idea why this doesn't work in the .cpp file. It works on
    // sensors, so there's probably a way to make it work here too.
    // - Joe, 2023 September 28
    Controller(const idClass& _ID): ID{_ID}
    {};

    idClass ID;

    SensorState getControllerSensorState(){return sensorState;}

};



#endif