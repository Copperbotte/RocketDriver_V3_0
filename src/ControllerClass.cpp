#include "ControllerClass.h"
#include <Arduino.h>

PropulsionController::PropulsionController(uint8_t setControllerNodeID, uint32_t setTargetValue, bool setNodeIDCheck) : controllerNodeID{setControllerNodeID}, targetValue{setTargetValue}, nodeIDCheck{setNodeIDCheck}
{
    // Instantiation stuff?
}

void PropulsionController::begin()
{
    if (nodeIDCheck)
    {
        // setup stuff?
    }
}

void PropulsionController::stateOperations()
{
    
}