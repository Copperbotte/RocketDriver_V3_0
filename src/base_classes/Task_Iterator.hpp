
#ifndef BASECLASS_TASK_ITERATOR_HPP_
#define BASECLASS_TASK_ITERATOR_HPP_

#include <Arduino.h>
#include "./AutoSequenceClass.h"

////////////////////////////////////////////////////////////////////////////////
// Base Task Iterator class.
//     A pure virtual class that defines a helper function used with the 
// runTasks template in src/OperationFunctionTemplates.h.
// 
//     These tasks have the same basic structure, but all have unique function
// names.  Maybe this is an artifact of the refactor.  If unneeded, please
// refactor it away again!
// - Joe, 2023 October 11

class Task_Iterator
{
public:
    // Runs object tasks in the main loop.
    virtual void runTasks(uint8_t& nodeIDReadIn,
        bool& outputOverride,
        AutoSequence& autoSequence) = 0;
};
#endif