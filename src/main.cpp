// RocketDriver V2.0 Propulsion Control and Data Acquisition - Embedded System Node Program
// Originally by Dan Morgan and Mat Arnold
// For Renegade, Beach Launch Team, Dan Morgan + Brandon Summers' personal machinations and more
//
//
// -------------------------------------------------------------
// Use top level define conditional to determine which system the code is operating
// Maintain definition header sets for a given propulsion system
#include "./main.hpp"

//#define PROPULSIONSYSNODEIDPRESET 2;     //NOT in use normally, for testing with the address IO register inactive

///// ADC /////
ADC* adc = new ADC();

// All ALARA boards need to be set to REF_1V2
ADC_REFERENCE ref0 = ADC_REFERENCE::REF_1V2;
ADC_REFERENCE ref1 = ADC_REFERENCE::REF_1V2;
uint8_t averages0 = 4;
uint8_t averages1 = 4;

MS5607 ALARAbaro;

uint32_t rocketDriverSeconds;
uint32_t rocketDriverMicros;


// Timer for setting main loop debugging print rate
elapsedMillis mainLoopTestingTimer;
elapsedMillis ezModeControllerTimer;
elapsedMillis commandExecuteTimer;
elapsedMillis shittyCANTimer;
elapsedMillis crashTimer;

//For use in doing serial inputs as CAN commands for testing
uint8_t fakeCANmsg; //CAN2.0 byte array, first 4 bytes are ID field for full extended ID compatibility
uint8_t fakeCanIterator = 0;

bool localNodeResetFlag = false; //flag to trigger register reset from commanded reset over CAN
bool abortHaltFlag; //creates halt flag that is a backup override of state machine, am I currently using it?
bool outputOverride = true; // initializes as true to block outputs until changed

///// NODE DECLARATION /////
//default sets to max nodeID intentionally to be bogus until otherwise set
ALARASN thisALARA;
uint8_t ALARAnodeID = 3;                      // ALARA hardware node address
uint8_t ALARAnodeIDfromEEPROM;            //nodeID read out of EEPROM
uint32_t ALARAnodeIDfromEEPROM_errorFlag;            //nodeID read out of EEPROM
bool nodeIDdeterminefromEEPROM;           //boolean flag for if startup is to run the nodeID detect read
uint32_t nodeIDdeterminefromEEPROM_errorFlag;
//uint8_t PropulsionSysNodeID = PROPULSIONSYSNODEIDPRESET;              //engine node = 2, prop node = 3, Pasafire node = 8
uint8_t PropulsionSysNodeID;              //engine node = 2, prop node = 3, Pasafire node = 8
uint8_t PropulsionSysNodeIDfromEEPROM;    //PropulsionSysNodeID read out of EEPROM
uint32_t PropulsionSysNodeIDfromEEPROM_errorFlag;    //PropulsionSysNodeID read out of EEPROM
uint32_t vehicleStatefromEEPROM_errorFlag;
uint32_t missionStatefromEEPROM_errorFlag;

const uint8_t configVerificationKey = 166; //upgrade to a map later

///// WATCHDOG SYSTEM /////
elapsedMillis propulsionControlWatchdog;                  // Watchdog timer that must be reset by ground control over bus to prevent an autovent
uint32_t propulsionControlWatchdogVentTime = 120000;   // 120 seconds in millis gives two minutes to reestablish control before autovent, DISABLE IN FLIGHT

/* ///// ADC /////
ADC* adc = new ADC();
 */
///// LED /////
elapsedMillis sinceLED;

///// CAN /////
CAN_message_t message;
CAN_message_t rxmsg;
CAN_message_t extended;
CAN_stats_t Can0stats;
bool CANSensorReportConverted = false;
bool NewCommandMessage{false};
bool NewConfigMessage{false};

CAN_filter_t wtf;

FlexCan3Controller Can2msgController;
SerialUSBController SerialUSBdataController;

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

bool startup{false}; // bool for storing if this is the first loop on startup, ESSENTIAL FOR STATE MACHINE OPERATION (maybe not anymore?)

uint32_t loopCount {0};// for debugging

// Set the global command, and global state
Command currentCommand{command_NOCOMMAND}; 
VehicleState currentVehicleState{VehicleState::passive};
VehicleState priorVehicleState{VehicleState::passive};
MissionState currentMissionState{MissionState::passive};
MissionState priorMissionState{MissionState::passive};
// SET "staticTest = true" FOR GROUND TESTING, "false" FOR FLIGHT!!!!!
bool staticTest = true;


commandMSG currentCommandMSG{};
configMSG currentConfigMSG{};

uint32_t vehicleStateAddressfromEEPROM_errorFlag;
uint32_t missionStateAddressfromEEPROM_errorFlag;

//AutoSequence stuff for main
int64_t currentCountdownForMain;

////// Set EEPROM addresses
// Change these up occasionally to reduce write cycle wear on the same bytes
// I could use EEPROM itself to store current start byte of my data and automate iterating this. Good idea for future upgrade.
uint16_t vehicleStateAddress1{4};
uint16_t vehicleStateAddress2{5};
uint16_t vehicleStateAddress3{6};
uint16_t missionStateAddress1{7};
uint16_t missionStateAddress2{8};
uint16_t missionStateAddress3{9};
uint16_t PropulsionSysNodeIDAddress1{16};
uint16_t PropulsionSysNodeIDAddress2{17};
uint16_t PropulsionSysNodeIDAddress3{18};
uint16_t nodeIDDetermineAddress1{19};
uint16_t nodeIDDetermineAddress2{20};
uint16_t nodeIDDetermineAddress3{21};
uint16_t nodeIDAddress1{22};
uint16_t nodeIDAddress2{23};
uint16_t nodeIDAddress3{24};

////////////////////////////////////////////////////////////////////////////////
FluidDefs fluidDefs;
SensorDefs sensorDefs;

////////////////////////////////////////////////////////////////////////////////
//     Main function call.  Is ran on power on once, followed by a while(true) 
// on loop.  
//-------------------------------------------------------//
void setup() {
    /////////////////////////////////////////////////////
    // -- Initialize hardware functions and globals -- //
    /////////////////////////////////////////////////////
    startup = true;   // Necessary to set startup to true for the code loop so it does one startup loop for the state machine before entering regular loop behavior
    Serial.begin(9600); // Value is arbitrary on Teensy, it will initialize at the MCU dictate baud rate regardless what you feed this
    Wire.begin();
    SPI.begin();

    #ifdef ALARAV2_1
    // ----- MUX Setups for ALARA -----
    // Board Addressing MUX
    MUXSetup(true, ALARA_DIGITAL_ADDRESS_1, ALARA_DIGITAL_ADDRESS_2, ALARA_DIGITAL_ADDRESS_3, ALARA_DIGITAL_ADDRESS_4);
    
    // MOVE NODEDETECTSHITHERE!!!
    // Check map for ALARASN configutation
    lookupALARASNmap(thisALARA, ALARAnodeID);

    // NOR Flash CS pin MUX
    MUXSetup(false, ALARA_NOR_S0, ALARA_NOR_S1, ALARA_NOR_S2);
    #endif
    ///// ----- Insert a board rev check to pin defines here, if it fails disable our GPIO? ------ //

    /////////////////////////////////
    // -- EEPROM initialization -- //
    /////////////////////////////////
    // I'm not sure why this is done at all..?

    // -----Read Last State off eeprom and update -----
    currentVehicleState = static_cast<VehicleState>(tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStateAddressfromEEPROM_errorFlag));
    currentMissionState = static_cast<MissionState>(tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStateAddressfromEEPROM_errorFlag));
    // Only write to EEPROM the node ID if manual ID define is present at top of Main
    //#ifdef PROPULSIONSYSNODEIDPRESET
    //tripleEEPROMwrite(static_cast<uint8_t>(2), PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3);
    //#endif
    //PropulsionSysNodeIDfromEEPROM = tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
    PropulsionSysNodeID = tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
    //nodeIDdeterminefromEEPROM = tripleEEPROMread(nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3, nodeIDdeterminefromEEPROM_errorFlag);
    startupStateCheck(currentVehicleState, currentCommand);
    // Set NewCommandMessage true so the command from startupStateCheck gets read by commandExecute
    NewCommandMessage = true;

    // ----- Run the Node ID Detection Function -----
    //PropulsionSysNodeID = NodeIDDetect(nodeID, nodeIDdeterminefromEEPROM, nodeIDfromEEPROM); // - OVERHAUL WITH NEW FUNCTION AND SYSTEM
    //PropulsionSysNodeID = PROPULSIONSYSNODEIDPRESET;       //For manually assigning NodeID isntead of the address read, make sure to comment out for operational use
    //PropulsionSysNodeID = thisALARA.propulsionSysNodeID;
    // Write 0 to byte for nodeIDDetermineAddress after reading it after a reset
    //tripleEEPROMwrite(0, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);
    //CHEATER OVERRIDE!!!!!
    //PropulsionSysNodeID = 8;

    //////////////////////////////
    // -- Initialize CAN Bus -- //
    //////////////////////////////
    //     This seems almost strange being here.  It should be with the rest of 
    // the CAN stuff near the bottom for clarity.

    // -----Initialize CAN-----
    vectorCommandBufferInitialize(32);  //number of vector elements memory is reserved for
    vectorConfigBufferInitialize(32); //number of vector elements memory is reserved for
    // CAN0 - FlexCAN 2.0 bus
    Can0.begin(CAN2busSpeed);

    // -----Initialize ADCs-----
    // Quick and dirty way to setup unique ADC use case on the LC/TC Teensy node
    #ifdef TEENSY3_X
        // Setting alt I2C pins because I used default I2C pins
        Wire.setSDA(8);
        Wire.setSCL(7);
        ref0 = ADC_REFERENCE::REF_3V3;
        ref1 = ADC_REFERENCE::REF_1V2;
        averages0 = 32;
        averages1 = 32;
    #endif
    MCUADCSetup(*adc, ref0, ref1, averages0, averages1);

    //////////////////////////////////
    // -- Initialize Sensors Bus -- //
    //////////////////////////////////

    //fluidDefs
    sensorDefs.pWaterGoesVroom = &fluidDefs.waterGoesVroom;

    // -----Run Valve PropulsionSysNodeID Check-----
    // ID Check verifies that the right devices are attached to the right ALARA.
    nodeIDCheck(valveArray, PropulsionSysNodeID);
    nodeIDCheck(pyroArray, PropulsionSysNodeID);
    nodeIDCheck(sensorDefs.sensorArray, PropulsionSysNodeID);
    nodeIDCheck(sensorDefs.sensorArray, 6); //Logger nodeID so it generates array for all sensors
    nodeIDCheck(sensorDefs.HPsensorArray, PropulsionSysNodeID);
    nodeIDCheck(sensorDefs.HPsensorArray, 6); //Logger nodeID so it generates array for all sensors
    //SensorNodeIDCheck(TCsensorArray, PropulsionSysNodeID);

    // -----Run Valve Setup-----
    setUp(valveArray, ALARA_HP_Array);
    setUp(pyroArray, ALARA_HP_Array);
    setUp(engineControllerArray);
    setUp(tankPressControllerArray);

    // -----Run AutoSequence Setup-----
    setUp(autoSequenceArray);
    
    // -----Run Sensor Setup -----
    setUp(sensorDefs.sensorArray);
    setUp(sensorDefs.HPsensorArray);
    setUp(sensorDefs.TCsensorArray);
    #ifdef TEENSY3_X
    sensorDefs.coldJunctionRenegade.begin();
    #endif

    // ----- Set Controller Dependent Sensor Settings -----
    controllerSensorSetup(valveArray, pyroArray, autoSequenceArray, sensorDefs.sensorArray, tankPressControllerArray, engineControllerArray);

    #ifdef ALARAV2_1
    pinModeExtended(ALARA_DIGITAL_ADDRESS_OE, OUTPUT);
    ALARAbaro.init(ALARA_BPS_CSN,OSR_1024);
    boardController.begin();
    #endif
    
    /////////////////////////
    // -- Start Can Bus -- //
    /////////////////////////
    //     The other half of the CAN bus initialization.  I'm not sure why this
    // is seperated from the first chunk.
    // 
    SerialUSBdataController.setPropStatusPrints(true);
    SerialUSBdataController.setPropCSVStreamPrints(false);
    Can0.startStats();
    Can0.setTxBufferSize(64);
}


/////////////////////
// -- Main Loop -- //
/////////////////////
//     This is the main loop of the driver.  Typically, these run on a while 
// loop forever.  I'm not sure if this is how it works under the hood here. -Joe
// 
// This function was filled with functions such as:
//Serial.println("Do I get past command execute?");
//Serial.println("Do I get past new config msg?");
//     In the event this is needed again, the following macro should be used 
// instead:
// 
//#define VERBOSE_PRINTS
#ifdef VERBOSE_PRINTS
#define DEBUG_SPRINT_PASS(_NAME) Serial.println("Do I get past" _NAME '?');
#else
#define DEBUG_SPRINT_PASS(_NAME)
#endif
// Debug Serial Print Pass. Duh what else could it mean ha ha ha ඞ


////////////////////////////////////////////////////////////////////////////////

void loop() 
{
    // Lazy "SensorTasks" for the RTD sensor
    if (sensorDefs.coldJunctionRenegade.ID.getNodeID() == PropulsionSysNodeID)
    {
        sensorDefs.coldJunctionRenegade.read();
    }

    //fakesensorShit(rocketDriverSeconds, rocketDriverMicros, &myTimeTrackingFunction);

    ///// Custom function for tracking miliseconds and seconds level system time for timestamping /////
    // let it run each loop in addition to when called to keep it synced up
    myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);

    // --- Read CAN bus and update current command ---
    if(CANread(Can0, configVerificationKey, NewConfigMessage, currentCommand, currentConfigMSG, PropulsionSysNodeID) && !startup) // do not execute on the first loop
    {
        //Serial.print("CAN Message Recieved: ");
        //Serial.println(currentCommand); //currently only does the command not any message
    }
    if(SerialAsCANread())
    {
        //Serial.println("Command Entered");
    }
    //Serial.println("Do I get to live?");
    

    ///// ------ MESSAGE PROCESSING BLOCK ----- /////  
    //pull next command message from buffer, if there is one
    if (!NewCommandMessage)
    {
        NewCommandMessage = readRemoveVectorBuffer(currentCommandMSG);
        currentCommand = currentCommandMSG.commandEnum;
    }
    //Serial.println("Do I get past new command message?");
    //process command
    //if (commandExecuteTimer >= 2000)
    //{
    commandExecute(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, NewCommandMessage, autoSequenceArray, sensorDefs.sensorArray, tankPressControllerArray, engineControllerArray);
    //Serial.println("Do I get past command execute?");
    //}
    //pull next config message from buffer, if there is one
    NewConfigMessage = readRemoveVectorBuffer(currentConfigMSG);
    //Serial.println("Do I get past new config msg?");
    //process config message
    configMSGread(currentConfigMSG, NewConfigMessage, valveArray, pyroArray, sensorDefs.sensorArray, autoSequenceArray, tankPressControllerArray, engineControllerArray, fluidDefs.waterGoesVroom);
    //Serial.println("Do I get past config msg read?");
    ///// ------------------------------------ /////  


    ////////////////////////////
    // -- Controller tasks -- //
    ////////////////////////////
    // State machine and controller think operations.
    // 
    if (ezModeControllerTimer >= 20) // 5 = 200Hz controller rate, 20 = 50Hz rate
    {
        // -----Process Commands Here-----
        vehicleStateMachine(currentVehicleState, priorVehicleState, currentCommand, boardController, autoSequenceArray, sensorDefs.sensorArray, tankPressControllerArray, engineControllerArray, fluidDefs.waterGoesVroom, abortHaltFlag, outputOverride);
        missionStateMachine(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, boardController, autoSequenceArray, staticTest, abortHaltFlag);
        
        #ifdef ALARAV2_1
        controllerDataSync(valveArray, pyroArray, autoSequenceArray, sensorDefs.sensorArray, tankPressControllerArray, engineControllerArray);
        #endif

        autoSequenceTasks(autoSequenceArray, PropulsionSysNodeID);
        tankPressControllerTasks(tankPressControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
        engineControllerTasks(engineControllerArray, PropulsionSysNodeID, IgnitionAutoSequence);
        controllerDeviceSync(currentVehicleState, priorVehicleState, currentCommand, valveArray, pyroArray, autoSequenceArray, sensorDefs.sensorArray, tankPressControllerArray, engineControllerArray, fluidDefs.waterGoesVroom, abortHaltFlag);
        //fluid sim run
        fluidDefs.waterGoesVroom.fluidSystemUpdate();
        //Serial.println("Is this Pizza's fault?");
        ezModeControllerTimer = 0;

        //ALARAbaro.update();
        //ALARAbaro.print_all();
    }

    DEBUG_SPRINT_PASS("Controller Tasks");
    
    // -----Advance needed controller system tasks (tank press controllers, ignition autosequence, . ..) ----- //
    // -----Advance needed propulsion system tasks (valve, pyro, sensors, . ..) ----- //

    ////////////////////////////////
    // -- Valve and Pyro tasks -- //
    ////////////////////////////////
    //     Updates the Valves and Pyros.  Interrupts are disabled for this
    // operation.
    // 

    cli(); // disables interrupts to ensure complete propulsion output state is driven

//valveTasks(valveArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
//pyroTasks(pyroArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
    runTasks(valveArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
    runTasks( pyroArray, PropulsionSysNodeID, outputOverride, *autoSequenceArray.at(0));
    //MUST KEEP HP OVERRIDE AFTER VALVE/PYRO TASKS
    #ifdef ALARAV2_1
    ALARAHPOverride(ALARA_HP_Array, outputOverride);
    #endif
    sei(); // reenables interrupts after propulsion output state set is completed
    DEBUG_SPRINT_PASS("Valve and Pyro Tasks");

    ////////////////////////////////
    // -- Sensor tasks -- //
    ////////////////////////////////
    // Updates sensors.
    // 

    sensorTasks(         sensorDefs.sensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros);
    ALARAHPsensorTasks(sensorDefs.HPsensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros, outputOverride);
    TCsensorTasks(     sensorDefs.TCsensorArray, *adc, PropulsionSysNodeID, rocketDriverSeconds, rocketDriverMicros);
    DEBUG_SPRINT_PASS("Sensor Tasks");

    /////////////////////////
    // -- EEPROM Update -- //
    /////////////////////////
    // Updates the EEPROM?? Why?
    // 

    // -----Update States on EEPROM -----
    // ONLY write if something new to write!!! Don't spam EEMPROM it will kill the memory bytes physically if overused
    if ((static_cast<uint8_t>(currentVehicleState)) != (tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStatefromEEPROM_errorFlag)))
    {
        tripleEEPROMwrite(static_cast<uint8_t>(currentVehicleState), vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3);
    }
    if ((static_cast<uint8_t>(currentMissionState)) != (tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStatefromEEPROM_errorFlag)))
    {
        tripleEEPROMwrite(static_cast<uint8_t>(currentMissionState), missionStateAddress1, missionStateAddress2, missionStateAddress3);
        /*   Serial.println("Does current vs prior MISSION state EEPROM protect work as expected? ");
        Serial.print(" priorMissionState : ");
        Serial.print(static_cast<uint8_t>(priorMissionState));
        Serial.print(" currentMissionState : ");
        Serial.println(static_cast<uint8_t>(currentMissionState)); */
    }
    
    // Reset function to reboot Teensy with internal reset register
    // Need to figure out how to rework using this feature with reworked ID system
    //TeensyInternalReset(localNodeResetFlag, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);


///// ----- All outgoing CAN2 messages managed here ----- /////
    // Run every loop
    if (shittyCANTimer >= 1000)
    {
        Can2msgController.setExternalStateChange(true); //cheater force for quasi messages, AM I USING THIS AT ALL RIGHT NOW???
        shittyCANTimer = 0;
    }
    
    Can2msgController.controllerTasks(Can0, currentVehicleState, currentMissionState, currentCommand, engineControllerArray, tankPressControllerArray, valveArray, pyroArray, sensorDefs.sensorArray, sensorDefs.HPsensorArray, autoSequenceArray, fluidDefs.waterGoesVroom, PropulsionSysNodeID);
    /*   Serial.println("Do I get past Can2 controllerTasks?");
    Can0stats = Can0.getStats();
    Serial.print("Can0stats.ringRxMax? ");
    Serial.println(Can0stats.ringRxMax);
    Serial.print("Can0stats.ringTxHighWater? ");
    Serial.println(Can0stats.ringTxHighWater); */

///// ----- Serial Print Functions ----- /////
    if (mainLoopTestingTimer >= 250)
    {
        SerialUSBdataController.propulsionNodeStatusPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, fluidDefs.waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorDefs.sensorArray, sensorDefs.HPsensorArray, PropulsionSysNodeID);
        SerialUSBdataController.propulsionNodeCSVStreamPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, fluidDefs.waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorDefs.sensorArray, PropulsionSysNodeID);
        mainLoopTestingTimer = 0; //resets timer to zero each time the loop prints
        Serial.print(" Crash Timer Millis: ");
        Serial.println(crashTimer);
    }

    // Resets the startup bool, DO NOT REMOVE
    startup = false;

}