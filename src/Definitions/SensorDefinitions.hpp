#ifndef SENSORDEFINITIONS_H
#define SENSORDEFINITIONS_H

#include "./Sensors/ALARAHPSensorClass.h"
#include "./Sensors/ALARAVRailSensorClass.h"
#include "./Sensors/EXTSensorClass.h"
#include "./Sensors/EXTDigitalDiffLCSensorClass.h"
#include "./TemperatureSensorClass.h"
#include <array>
#include "./ALARApinDefines.h"
#include "./fluidSimulationDefinitions.h"

#ifdef RENEGADESF

// define number of sensors here
#define NUM_SENSORS 23

////////////////////////////////////////////////////////////////////////////////
//     Is this the right place for magic numbers?  This looks like calibration
// data, straight from the excel doc.  Everything is in one convenient place
// so I'm assuming yes. - Joe, 2023 October 14

// initialize all sensor objects here
// Renegade SF Stand
//Non LC normal sensor objects
// Node 2
struct SensorDefs
{

FluidSystemSimulation *pWaterGoesVroom;

EXT_SENSOR ChamberPT2{              idClass{ 52, 2}, ALARA_ANALOG_IN7, &pWaterGoesVroom, 10, 100, 500, 0.0196, -102.94}; //  6
EXT_SENSOR ChamberPT1{              idClass{ 50, 2}, ALARA_ANALOG_IN8, &pWaterGoesVroom, 10, 100, 500, 0.0195, -128.88}; //  7
EXT_SENSOR FuelInletPropSidePT{     idClass{ 58, 2}, ALARA_ANALOG_IN6, &pWaterGoesVroom,  5,  10, 100, 0.0185, -125.74}; //  8
EXT_SENSOR FuelInjectorPT{          idClass{ 54, 2}, ALARA_ANALOG_IN4, &pWaterGoesVroom, 10, 100, 100, 0.0196, -123.27}; //  9
EXT_SENSOR LoxInletPropSidePT{      idClass{ 60, 2}, ALARA_ANALOG_IN5, &pWaterGoesVroom,  5,  10, 100, 0.0196, -128.58}; // 10
EXT_SENSOR MVPneumaticsPT{          idClass{ 56, 2}, ALARA_ANALOG_IN3, &pWaterGoesVroom,  5,   5,  10, 0.0193, -125.56}; // 11
// Node 3
EXT_SENSOR DomeRegFuelPT{           idClass{ 74, 3}, ALARA_ANALOG_IN1, &pWaterGoesVroom,  5,   5,  10, 0.0196, -127.95}; // 12
EXT_SENSOR DomeRegLoxPT{            idClass{ 76, 3}, ALARA_ANALOG_IN2, &pWaterGoesVroom,  5,   5,  10, 0.0194, -134.95}; // 13
EXT_SENSOR FuelTankPT1{             idClass{ 62, 3}, ALARA_ANALOG_IN3, &pWaterGoesVroom,  5,  50, 100, 0.0192, -125.04}; // 14
EXT_SENSOR FuelTankPT2{             idClass{ 64, 3}, ALARA_ANALOG_IN8, &pWaterGoesVroom,  5,  10, 100, 0.0194, -125.08}; // 14
EXT_SENSOR LoxTankPT1{              idClass{ 66, 3}, ALARA_ANALOG_IN4, &pWaterGoesVroom,  5,  50, 100, 0.0192, -122.78}; // 15
EXT_SENSOR LoxTankPT2{              idClass{ 68, 3}, ALARA_ANALOG_IN7, &pWaterGoesVroom,  5,  50, 100, 0.0191, -126.90}; // 15
EXT_SENSOR HiPressFuelPT{           idClass{ 70, 3}, ALARA_ANALOG_IN5, &pWaterGoesVroom,  5,  10,  50, 0.0967, -623.11}; // 16
EXT_SENSOR HiPressLoxPT{            idClass{ 72, 3}, ALARA_ANALOG_IN6, &pWaterGoesVroom,  5,  10,  50, 0.0981, -630.47}; // 17

//FAKESHIT
EXT_SENSOR FakeChamberPT1{          idClass{150, 2},              41, &pWaterGoesVroom, simulatedInput}; // 7
EXT_SENSOR FakeFuelLinePT{          idClass{158, 2},              32, &pWaterGoesVroom, simulatedInput}; // 8
EXT_SENSOR FakeLoxLinePT{           idClass{160, 2},              22, &pWaterGoesVroom, simulatedInput}; // 10
EXT_SENSOR FakeFuelTankPT{          idClass{162, 3},              31, &pWaterGoesVroom, simulatedInput}; // 14
EXT_SENSOR FakeLoxTankPT{           idClass{166, 3},              21, &pWaterGoesVroom, simulatedInput}; // 15
EXT_SENSOR FakeHiPressPT{           idClass{170, 3},              11, &pWaterGoesVroom, simulatedInput}; // 16

//LC Sensors
DIG_LC_SENSOR ThrustMountLoadCell1{ idClass{ 32, 4}, A14, A15, &pWaterGoesVroom, 100, 1000, 10000, 100}; // 0,1
DIG_LC_SENSOR ThrustMountLoadCell2{ idClass{ 38, 4}, A16, A17, &pWaterGoesVroom, 100, 1000, 10000, 100}; // 2,3
DIG_LC_SENSOR ThrustMountLoadCell3{ idClass{ 44, 4}, A18, A19, &pWaterGoesVroom, 100, 1000, 10000, 100}; // 4,5

// Temp Sensors
RTD_BREAKOUT coldJunctionRenegade{  idClass{ 99, 4}, 24, 3};
THERMOCOUPLE EngineChamberWallTC{   idClass{100, 4}, A0, A1, T_Type, &coldJunctionRenegade};
THERMOCOUPLE EngineThroatWallTC{    idClass{102, 4}, A2, A3, T_Type, &coldJunctionRenegade};
THERMOCOUPLE EngineNozzleExitWallTC{idClass{104, 4}, A4, A5, T_Type, &coldJunctionRenegade};  //Need to move I2C lines to clear A4, A5 pins
THERMOCOUPLE LoxTankLowerTC{        idClass{106, 4}, A6, A7, T_Type, &coldJunctionRenegade};
THERMOCOUPLE LoxTankMidTC{          idClass{108, 4}, A8, A9, T_Type, &coldJunctionRenegade};
THERMOCOUPLE LoxTankUpperTC{        idClass{110, 4}, A10, A11, T_Type, &coldJunctionRenegade};

// HP Channel Sensors
// Node 2
ALARAHP_SENSOR RenegadeEngineHP1{   idClass{121, 2}, ALARA_HIGHPOWER_ANALOGREAD1 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP2{   idClass{122, 2}, ALARA_HIGHPOWER_ANALOGREAD2 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP3{   idClass{123, 2}, ALARA_HIGHPOWER_ANALOGREAD3 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP4{   idClass{124, 2}, ALARA_HIGHPOWER_ANALOGREAD4 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP5{   idClass{125, 2}, ALARA_HIGHPOWER_ANALOGREAD5 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP6{   idClass{126, 2}, ALARA_HIGHPOWER_ANALOGREAD6 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP7{   idClass{127, 2}, ALARA_HIGHPOWER_ANALOGREAD7 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP8{   idClass{128, 2}, ALARA_HIGHPOWER_ANALOGREAD8 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP9{   idClass{129, 2}, ALARA_HIGHPOWER_ANALOGREAD9 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP10{  idClass{130, 2}, ALARA_HIGHPOWER_ANALOGREAD10, 0.0006,1.7800};
// Node 3
ALARAHP_SENSOR RenegadePropHP1{     idClass{131, 3}, ALARA_HIGHPOWER_ANALOGREAD1 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP2{     idClass{132, 3}, ALARA_HIGHPOWER_ANALOGREAD2 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP3{     idClass{133, 3}, ALARA_HIGHPOWER_ANALOGREAD3 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP4{     idClass{134, 3}, ALARA_HIGHPOWER_ANALOGREAD4 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP5{     idClass{135, 3}, ALARA_HIGHPOWER_ANALOGREAD5 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP6{     idClass{136, 3}, ALARA_HIGHPOWER_ANALOGREAD6 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP7{     idClass{137, 3}, ALARA_HIGHPOWER_ANALOGREAD7 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP8{     idClass{138, 3}, ALARA_HIGHPOWER_ANALOGREAD8 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP9{     idClass{139, 3}, ALARA_HIGHPOWER_ANALOGREAD9 , 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP10{    idClass{140, 3}, ALARA_HIGHPOWER_ANALOGREAD10, 0.0006,1.7800};

// ALARA Renegade Engine and Prop Node HP sensor array
typedef std::array<ALARAHP_SENSOR*, 20> HPsensorArray_type;
HPsensorArray_type HPsensorArray{
    &RenegadeEngineHP1, &RenegadeEngineHP2, &RenegadeEngineHP3, &RenegadeEngineHP4, &RenegadeEngineHP5,
    &RenegadeEngineHP6, &RenegadeEngineHP7, &RenegadeEngineHP8, &RenegadeEngineHP9, &RenegadeEngineHP10,
    &RenegadePropHP1, &RenegadePropHP2, &RenegadePropHP3, &RenegadePropHP4, &RenegadePropHP5,
    &RenegadePropHP6, &RenegadePropHP7, &RenegadePropHP8, &RenegadePropHP9, &RenegadePropHP10
};

// Sensor Array including Renegade SF only
typedef std::array<Sensor*, NUM_SENSORS> sensorArray_type;
sensorArray_type sensorArray{
    &ThrustMountLoadCell1, &ThrustMountLoadCell2, &ThrustMountLoadCell3,
    &ChamberPT2, &ChamberPT1, &FuelInletPropSidePT, &FuelInjectorPT,
    &LoxInletPropSidePT, &MVPneumaticsPT, &DomeRegFuelPT, &DomeRegLoxPT,
    &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressFuelPT, &HiPressLoxPT,
    &FakeChamberPT1, &FakeFuelLinePT, &FakeLoxLinePT, &FakeFuelTankPT, &FakeLoxTankPT, &FakeHiPressPT
};

typedef std::array<THERMOCOUPLE*, 6> TCsensorArray_type;
TCsensorArray_type TCsensorArray{
    &EngineChamberWallTC, &EngineThroatWallTC, &EngineNozzleExitWallTC,
    &LoxTankLowerTC, &LoxTankMidTC, &LoxTankUpperTC
};

};

#endif

#endif