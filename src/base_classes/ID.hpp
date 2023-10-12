
#ifndef BASECLASS_NODE_ID_CHECK_HPP_
#define BASECLASS_NODE_ID_CHECK_HPP_

#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////
// Base Node ID Check class.
// - Joe, 2023 October 11
class idClass
{
private:
    const uint32_t _ID;
    const uint8_t _NodeID;

    bool _nodeIDCheck = false; // Whether this object should operate on this node

public:
    // Initializer constructor.
    idClass(uint32_t ID,  uint8_t NodeID):
                 _ID{ID}, _NodeID{NodeID}, _nodeIDCheck{false}
    {}

    // Copy constructor.
    idClass(idClass &Other):
        _ID{Other._ID}, _NodeID{Other._NodeID}, _nodeIDCheck{Other._nodeIDCheck}
    {}


    uint32_t getID() const {return _ID;}
    uint8_t getNodeID() const {return _NodeID;}

    bool getNodeIDCheck(){return _nodeIDCheck;}
    void setNodeIDCheck(bool updatedNodeIDCheck) {_nodeIDCheck = updatedNodeIDCheck;} // set the Node ID Check bool function
};

#endif