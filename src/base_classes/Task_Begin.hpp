
#ifndef BASECLASS_TASK_BEGIN_HPP_
#define BASECLASS_TASK_BEGIN_HPP_

////////////////////////////////////////////////////////////////////////////////
// Base Task Begin class.
// A pure virtual class that defines a common Begin function in most objects.
// Used within the main loop via runTasks.
// - Joe, 2023 October 11
class Task_Begin
{
public:
    virtual void begin() = 0;
};

////////////////////////////////////////////////////////////////////////////////
// Base Task Begin class for objects that access pins.
// A pure virtual class that defines a common Begin function in most objects.
// Used within the main loop via runTasks.
//     Maybe there's a way to do this with a templated base class, or some other
// kind of clever constructor?
// - Joe, 2023 October 11
class Task_Begin_Pins
{
public:
    virtual void begin(uint8_t pinArrayIn[][11]) = 0;
};

#endif