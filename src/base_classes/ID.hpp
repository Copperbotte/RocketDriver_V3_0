
#ifndef BASECLASS_NODE_ID_CHECK_HPP_
#define BASECLASS_NODE_ID_CHECK_HPP_

#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////
// Base Node ID Check class.
//     ID is the unique ID of this object, while NodeID is which ALARA this 
// object lies on.
// - Joe, 2023 October 11
// 
// Constructors:
// - idClass(ID, NodeID)
// - idClass(const &idClass)
// 
// Functions:
// - uint32_t getID() - returns the ID.
// - uint8_t getNodeID() - returns the Node ID.
// - bool getNodeIDCheck() - returns if this ID passed the ID check test.
// - void setNodeIDCheck(bool updatedNodeIDCheck) - Sets if this ID passed the ID check test.  DO NOT USE THIS.
class idClass
{
private:
    const uint32_t _ID;
    const uint8_t _NodeID;

    bool _nodeIDCheck = false; // Whether this object should operate on this node

public:
    // Initializer constructor.
    // Inputs:
    // - uint32_t ID    // The unique ID of this sensor
    // - uint8_t NodeID // The unique ID of the node this sensor is attached to
    idClass(uint32_t ID,  uint8_t NodeID):
                 _ID{ID}, _NodeID{NodeID}, _nodeIDCheck{false}
    {}

    // Copy constructor.
    // Inputs:
    // - const idClass &Other // A reference to an existing idClass.
    // 
    // Note: Please do not use this to duplicate an ID class.  
    idClass(const idClass &Other):
        _ID{Other._ID}, _NodeID{Other._NodeID}, _nodeIDCheck{Other._nodeIDCheck}
    {}

    // Gets this ID's id.
    // Inputs:
    // - nothing.
    //
    // Returns:
    // - uint32_t ID
    uint32_t getID() const {return _ID;}

    // Gets this ID's Nodeid.
    // Inputs:
    // - nothing.
    //
    // Returns:
    // - uint8_t NodeID
    uint8_t getNodeID() const {return _NodeID;}

    // Gets this ID's nodeIDCheck flag, indicating if it passed the nodeIDcheck.
    // Inputs:
    // - nothing.
    //
    // Returns:
    // - bool nodeIDCheck
    bool getNodeIDCheck(){return _nodeIDCheck;}

    // Sets this ID's nodeIDCheck flag, indicating if it passed the nodeIDcheck.
    // ONLY nodeIDCheck within src/OperationFunctionTemplates.h should use this function.
    // Inputs:
    // - bool nodeIDCheck
    //
    // Returns:
    // - nothing.
    void setNodeIDCheck(bool updatedNodeIDCheck) {_nodeIDCheck = updatedNodeIDCheck;} // set the Node ID Check bool function
};

#endif