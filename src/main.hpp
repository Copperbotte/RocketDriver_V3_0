#ifndef MAIN_HPP
#define MAIN_HPP

////////////////////////////////////////////////////////////////////////////////
//     Main header file.  This should contain all headers referenced by main, so
// main only imports this file.  This file should not be included anywhere else.
// Used to keep main clean and readable.
//
// Try to keep the includes sorted by "group", then by "type", then alphabetical.
// - Joe Kessler, 2023 October 12

// Definitions
#include "./Definitions/ALARABoardControllerDefinitions.h"
#include "./Definitions/ALARASensorControllerDefinitions.h"
#include "./Definitions/EngineControllerDefinitions.h"
#include "./Definitions/TankPressControllerDefinitions.h"

#include "./Definitions/SensorDefinitions.hpp"

#include "./Definitions/PyroDefinitions.hpp"
#include "./Definitions/ValveDefinitions.hpp"

#include "ALARApinDefines.h"
#include "AutoSequenceDefinitions.h"


// Classes
#include "./Controllers/ALARABoardControllerClass.h"

// IO
#include "ms5607/ms5607.h" // This is the barometer!

#include "CANRead.h"
#include "CANWrite.h"

#include "FlexCAN3Controller.h"
#include "SerialUSBController.h"

#include "extendedIO/extendedIO.h"

// Trying to figure out RTC stuff with these libs
// Unknown timescale on that comment. Probably dan, before 2023.
// - Joe, 2023 Oct 12
#include <TimeLib.h>
#include <DS1307RTC.h>

// Other stuff
#include "ALARAUtilityFunctions.h"
#include "ControlFunctions.h"
#include "fluidSystemSimulation.h"
#include "OperationFunctionTemplates.h"

// ----- "COTS" includes ----- //
// "Commercial Off The Shelf"
#include "Adafruit_MCP9808.h" //  Where is this file? - Joe, 2023 July 15
#include <ADC.h>
#include <ADC_util.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <FlexCAN.h>
#include <InternalTemperature.h>
#include <IntervalTimer.h>
#include <kinetis_flexcan.h>
#include <WireKinetis.h>
#include <Wire.h>

// Standard Library
#include <array>
#include <string>
#include <list>
#include <unordered_map>
using std::string; // eww - Joe 2023 Oct 12


#endif