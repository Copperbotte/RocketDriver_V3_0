#include "PropControllerClass.h"
#include <Arduino.h>

PropulsionController::PropulsionController(uint8_t setControllerNodeID, uint32_t setTargetValue, bool setNodeIDCheck):
    ID{PROPCONTROLLERID, setControllerNodeID}, targetValue{setTargetValue}
{
    ID.setNodeIDCheck(setNodeIDCheck);
    // Instantiation stuff?
}

void PropulsionController::begin()
{
    if (ID.getNodeIDCheck())
    {
        // setup stuff?
    }
}

void PropulsionController::stateOperations()
{
    
}