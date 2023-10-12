#ifndef OPERATIONFUNCTIONTEMPLATES_H
#define OPERATIONFUNCTIONTEMPLATES_H

#include <array>
//#include <bitset>
//#include <FlexCAN.h>
//#include <ADC.h>
#include "ToMillisTimeTracker.h"

// This contains some of the functions to be used during operations they are templates, and so defined in the header. BEWARE

// Generic setup loop.
// Accepts an std::array reference of things that have Task_Begin.begin().
template <typename T, std::size_t size>
void setUp(const std::array<T, size>& Array)
{
    // iterate through valve array and run the stateOperations method
    for(auto item : Array)
    {
        item->begin();
    }
}

// Generic setup loop.
// Accepts an std::array reference of things that have Task_Begin.begin(uint8_t pinArrayIn[][11]).
template <typename T, std::size_t size>
void setUp(const std::array<T, size>& Array, uint8_t pinArrayIn[][11])
{
    // iterate through valve array and run the stateOperations method
    for(auto item : Array)
    {
        item->begin(pinArrayIn);
    }
}

// Sets an object in the array's NodeIDCheck to true if its NodeID matches the input id.
template <typename T, std::size_t size>
void nodeIDCheck(const std::array<T, size>& Array, uint8_t nodeIDfromMain)
{
    for (auto item : Array)
    {
        if (item->ID.getNodeID() == nodeIDfromMain)
        {
            item->ID.setNodeIDCheck(true);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//     Below here are a series of task functions.  runTasks should be generic, 
// but I didn't find a clever way for all of the tasks to be ran with the same 
// function at the time.  It was late at night, it might be easy.  Please keep
// in mind performance due to these being a funcitonal approach running 
// effectively a Map() on a series of functions.  Try to avoid using entire 
// arrays as arguments to a function as the array will get packed onto the heap
// during each function call.  I think.  Needs testing. - Joe, 2023 Oct 12

//  CALL THIS FUNCTION EVERY LOOP 
    // This function takes the array of pointers that point to the valve objects, and then calls the .stateOperations() method for each valve
    // Make sure valveArray is an array of pointers, as defined in ValveDefinitions.h
template <typename T, std::size_t size>
void runTasks(const std::array<T, size>& Array, uint8_t& nodeIDReadIn, bool& outputOverride, AutoSequence& mainAutoSequence)
{
    if (!outputOverride)    //bool will block all stateOps
    {
        // iterate through the array and run the lambda method
        for(auto item : Array)
        {
            item->runTasks(nodeIDReadIn, outputOverride, mainAutoSequence);
        }
    }
}

template <typename T, std::size_t size>
void tankPressControllerTasks(const std::array<T, size>& tankPressControllerArray, uint8_t& nodeIDReadIn, AutoSequence& ignitionAutoSequenceRef)
{
    // iterate through valve array and run the stateOperations method
    for(auto tankPressController : tankPressControllerArray)
    {
    
        if (tankPressController->ID.getNodeID() == nodeIDReadIn)
        {
            tankPressController->setCurrentAutosequenceTime(ignitionAutoSequenceRef.getCurrentCountdown());
            tankPressController->stateOperations();
        }
    }
}

template <typename T, std::size_t size>
void engineControllerTasks(const std::array<T, size>& engineControllerArray, uint8_t& nodeIDReadIn, AutoSequence& ignitionAutoSequenceRef)
{
    // iterate through valve array and run the stateOperations method
    for(auto engineController : engineControllerArray)
    {
    
        if (engineController->ID.getNodeID() == nodeIDReadIn)
        {
            engineController->setCurrentAutosequenceTime(ignitionAutoSequenceRef.getCurrentCountdown());
            engineController->stateOperations();
        }
    }
}

template <typename T, std::size_t size>
void autoSequenceTasks(const std::array<T, size>& autoSequenceArray, uint8_t& nodeIDReadIn)
{
    // iterate through valve array and run the stateOperations method
    for(auto autoSequence : autoSequenceArray)
    {
        
    if (autoSequence->getHostNodeID() == nodeIDReadIn)
        {
            autoSequence->stateOperations();
        }
    }
}

template <typename T, std::size_t size>
//void sensorTasks(const std::array<T, size>& sensorArray, ADC*adc, uint32_t& secondsRD,uint32_t& microsecondsRD, uint8_t& nodeIDReadIn)
void sensorTasks(const std::array<T, size>& sensorArray, ADC& adc, uint8_t& nodeIDReadIn, uint32_t& rocketDriverSeconds, uint32_t&  rocketDriverMicros)
{
    // iterate through valve array and run the stateOperations method
    for(auto sensor : sensorArray)
    {
    
        if (sensor->ID.getNodeID() == nodeIDReadIn)
        {
            sensor->stateOperations();
            //Serial.print("LoopRan");
            sensor->read(adc);
            myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
            sensor->setSYSTimestamp(rocketDriverSeconds, rocketDriverMicros);
            //sensor->linearConversion();
        }
/*         else if (nodeIDReadIn == 6) //shitty way to make logger node only convert
        {
            sensor->linearConversion();
        } */
    }
}

template <typename T, std::size_t size>
//void sensorTasks(const std::array<T, size>& sensorArray, ADC*adc, uint32_t& secondsRD,uint32_t& microsecondsRD, uint8_t& nodeIDReadIn)
void ALARAHPsensorTasks(const std::array<T, size>& sensorArray, ADC& adc, uint8_t& nodeIDReadIn, uint32_t& rocketDriverSeconds, uint32_t&  rocketDriverMicros, bool outputOverrideIn)
{
    // iterate through valve array and run the stateOperations method
    for(auto sensor : sensorArray)
    {
    
        if (sensor->ID.getNodeID() == nodeIDReadIn)
        {
            sensor->stateOperations();
            //Serial.println("LoopRan for HP sensor tasks: ");
            sensor->read(adc);
            sensor->setDeenergizeOffset(adc, outputOverrideIn);
            myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
            sensor->setSYSTimestamp(rocketDriverSeconds, rocketDriverMicros);
            //sensor->linearConversion();
        }
/*         else if (nodeIDReadIn == 6) //shitty way to make logger node only convert
        {
            sensor->linearConversion();
        } */
    }
}

template <typename T, std::size_t size>
//void sensorTasks(const std::array<T, size>& sensorArray, ADC*adc, uint32_t& secondsRD,uint32_t& microsecondsRD, uint8_t& nodeIDReadIn)
void TCsensorTasks(const std::array<T, size>& TCsensorArray, ADC& adc, uint8_t& nodeIDReadIn, uint32_t& rocketDriverSeconds, uint32_t&  rocketDriverMicros)
{
    // iterate through valve array and run the stateOperations method
    for(auto sensor : TCsensorArray)
    {
        if (sensor->ID.getNodeID() == nodeIDReadIn)
        {
            sensor->stateOperations();
            //Serial.println("LoopRan for HP sensor tasks: ");
            sensor->read(adc);
            myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
            sensor->setSYSTimestamp(rocketDriverSeconds, rocketDriverMicros);
            //sensor->linearConversion();
        }
    }
}

void fakesensorShit(uint32_t& rocketDriverSeconds, uint32_t& rocketDriverMicros, void (*myTimeTrackingFunction)(uint32_t, uint32_t))
{
    // iterate through sensor array and run begin
    //for(auto sensor : sensorArray)
    //{
        //sensor->begin();
        //sensor->setSYSTimestamp(secondsRD, microsecondsRD);
        myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);
        Serial.println("rocketDriverSeconds");
        Serial.println(rocketDriverSeconds);
        Serial.println("rocketDriverMicros");
        Serial.println(rocketDriverMicros);
        //Serial.print("LoopRan");
    //}
}



#endif