#include "ALARAUtilityFunctions.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>
#include "extendedIO/extendedIO.h"

// ----- Teensy Internal Reset -----
// Teensy 3.5/3.6 MCU restart register definitions
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

void tripleEEPROMwrite(uint8_t byteToWrite, uint32_t byteAddress1, uint32_t byteAddress2, uint32_t byteAddress3)
{
    // -----Update State on EEPROM -----
  cli(); // disables interrupts to protect write command
  EEPROM.update(byteAddress1, byteToWrite);                                 // Never use .write()
  EEPROM.update(byteAddress2, byteToWrite);                                 // Never use .write()
  EEPROM.update(byteAddress3, byteToWrite);                                 // Never use .write()
  sei(); // reenables interrupts after write is completed
}

uint8_t tripleEEPROMread(uint32_t byteAddress1, uint32_t byteAddress2, uint32_t byteAddress3, uint32_t &errorFlag)
{
  uint8_t byteReadFrom1;
  uint8_t byteReadFrom2;
  uint8_t byteReadFrom3;
  byteReadFrom1 = EEPROM.read(byteAddress1);
  byteReadFrom2 = EEPROM.read(byteAddress2);
  byteReadFrom3 = EEPROM.read(byteAddress3);

// Error handling and voting for non matching results
// errorFlag = 0 means no error, 
// otherwise bytes 0,1,2 are combined to return to mixed result in totality
// bits above 24 of EEPROM byte info are set as boolean digits to flag the non matching bytes

  if (byteReadFrom1 == byteReadFrom2 && byteReadFrom1 == byteReadFrom3)
  {
    errorFlag = 0;           
    return byteReadFrom1;
  }
  else if (byteReadFrom1 == byteReadFrom2)
  {
    errorFlag = (4 >> 24) + (byteReadFrom3 >> 16);
    return byteReadFrom1;
  }
  else if (byteReadFrom1 == byteReadFrom3)
  {
    errorFlag = (2 >> 24) + (byteReadFrom2 >> 8);
    return byteReadFrom1;
  }
  else if (byteReadFrom2 == byteReadFrom3)
  {
    errorFlag = (1 >> 24) + (byteReadFrom1 >> 0);
    return byteReadFrom2;
  }
  else
    errorFlag = (7 >> 24) + (byteReadFrom3 >> 16) + (byteReadFrom2 >> 8) + (byteReadFrom1 >> 0);
    return byteReadFrom1;
}

bool readOnlyMUX(uint8_t pinToReadMUX, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2, uint8_t pinMUX_A) // SN74CB3Q3251 MUX pin read operation for use only when MUX has OE tied to GND
{
  digitalWriteExtended(pinMUX_S0, (pinToReadMUX >> 0) & 1L);
  digitalWriteExtended(pinMUX_S1, (pinToReadMUX >> 1) & 1L);
  digitalWriteExtended(pinMUX_S2, (pinToReadMUX >> 2) & 1L);
  return digitalReadExtended(pinMUX_A);
}

void SPI_CS_MUX(uint8_t pinToWriteMUX, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2) // SN74CB3Q3251 MUX pin output operation for use as multiplexed CS pins
{
  digitalWriteExtended(pinMUX_S0, (pinToWriteMUX >> 0) & 1L);
  digitalWriteExtended(pinMUX_S1, (pinToWriteMUX >> 1) & 1L);
  digitalWriteExtended(pinMUX_S2, (pinToWriteMUX >> 2) & 1L);
}

void MUXSetup(bool MUX_Input, uint8_t pinMUX_S0, uint8_t pinMUX_S1, uint8_t pinMUX_S2, uint8_t pinMUX_A = 0)       // SN74CB3Q3251 MUX pin output setup function, set MUX_Input = false and leave pinMUX_A empty for output MUX
{
    pinModeExtended(pinMUX_S0, OUTPUT);
    pinModeExtended(pinMUX_S1, OUTPUT);
    pinModeExtended(pinMUX_S2, OUTPUT);
    if (MUX_Input)                  // Use MUX_Input boolean to set the input pin only if this MUX will be used as input, output MUX doesn't use
    {
      pinModeExtended(pinMUX_A, INPUT);
    }
}


// -------------------------------------------------------------
uint8_t NodeIDDetect(uint8_t pinToReadMUXNodeID, uint8_t pinMUX_S0NodeID, uint8_t pinMUX_S1NodeID, uint8_t pinMUX_S2NodeID, uint8_t pinMUX_ANodeID,
                    uint16_t nodeIDDetermineAddress1, uint16_t nodeIDDetermineAddress2, uint16_t nodeIDDetermineAddress3, uint16_t NodeIDAddress1, uint16_t NodeIDAddress2, uint16_t NodeIDAddress3, uint8_t nodeID, bool nodeIDdetermine, uint8_t nodeIDfromEEPROM1, uint8_t nodeIDfromEEPROM2, uint8_t nodeIDfromEEPROM3)       //Function to run the nodeID hardware addressing
{
  uint8_t NodeIDAddressRead = 0;
  uint8_t NodeIDAddressReadEEPROM = 0;
  uint8_t ALARAnodeIDfromEEPROM = 0;
  uint32_t ALARAnodeIDfromEEPROM_errorFlag = 0;

  for (size_t i = 0; i < 8; i++)
  {
    NodeIDAddressRead = readOnlyMUX(i, pinMUX_S0NodeID, pinMUX_S1NodeID,  pinMUX_S2NodeID, pinMUX_ANodeID) << i;
  }
  if (NodeIDAddressRead == 0 || NodeIDAddressRead == 255)
  {
    // reset the MUX outputs to Inputs
    pinModeExtended(pinMUX_S0NodeID, INPUT);
    pinModeExtended(pinMUX_S1NodeID, INPUT);
    pinModeExtended(pinMUX_S2NodeID, INPUT);
    // read the MUX pins as plain digital input addressing pins
    NodeIDAddressRead = 0;  //reset the nodeID to all zeroes in case of MUX read of 255, redundant step to make sure previous read bits are cleared
    NodeIDAddressRead = (digitalReadExtended(pinMUX_S0NodeID) >> 0) + (pinMUX_S1NodeID >> 1) + (pinMUX_S2NodeID >> 2) + (pinMUX_ANodeID >> 3);

  }
  // if the nodeIDdetermine bool is true that means read the address and write that as the address to EEPROM
  if (nodeIDdetermine)
    {
    // writes the nodeIDdetermine bool to false to reset it
    tripleEEPROMwrite(0, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);
    // write new node address data to EEPROM below
    tripleEEPROMwrite(NodeIDAddressRead, NodeIDAddress1, NodeIDAddress2, NodeIDAddress3);
    }
  else
    // if nodeIDdetermine is not true, then proceed with reading from EEPROM and comparing results
    ALARAnodeIDfromEEPROM = tripleEEPROMread(NodeIDAddress1, NodeIDAddress2, NodeIDAddress3, ALARAnodeIDfromEEPROM_errorFlag);
    if (ALARAnodeIDfromEEPROM == NodeIDAddressRead)
    {
      return NodeIDAddressRead;
    }
    else
      
      
      
      return NodeIDAddressRead;
}
// -------------------------------------------------------------


// ----- Teensy Internal Reset -----
void TeensyInternalReset (bool& localNodeResetFlagIn, uint8_t addressIn1, uint8_t addressIn2, uint8_t addressIn3)
{
  if (localNodeResetFlagIn)
  {
    tripleEEPROMwrite(1, addressIn1, addressIn2, addressIn3);
    WRITE_RESTART(0x5FA0004);
  }
  else; //nothing else but it feels right
}

// ----- Software PWM Function -----
// behavior is not reliable when there are delays
elapsedMicros fakePWMMicros;
uint32_t dutyOnMicros;
uint32_t periodMicros;

void analogWriteSoft(uint8_t outputPin, bool outputState, uint32_t freqIN, uint16_t dutyCycleIn)
{
    periodMicros = 1000000/freqIN;
    dutyOnMicros = (dutyCycleIn * periodMicros)/256;
    if (outputState)
    {
        if (fakePWMMicros >= (periodMicros))
        {
            digitalWriteExtended(outputPin,1);
            fakePWMMicros = 0;
        }
        if (fakePWMMicros >= dutyOnMicros)
        {
            digitalWriteExtended(outputPin,0);
        }
    } 
}

float float_from32bits(uint32_t f)
{
    //static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");  //shouldn't be needed on platform where float is always 4 bytes
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}
float uint32_fromfloat(float u)
{
    //static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");  //shouldn't be needed on platform where float is always 4 bytes
    uint32_t ret;
    std::memcpy(&ret, &u, sizeof(uint32_t));
    return ret;
}


uint32_t arrayIndexFirstValue = 0;
uint32_t sizeInputArrayInsert = 0;
uint32_t sizeInputArrayInsert1 = 0;
uint32_t sizeInputArrayInsert2 = 0;
uint32_t arrayMostRecentPositionInsert = 0;
// utility function for running a rolling array
// float array version - !!!!! Not Protected from if you put negative signed floats for the index values !!!!!
void writeToRollingArray(float rollingArray[], float newInputArrayValue, uint32_t numberSizeIndex = 1)
{
  // Updated format to allow arbitrary number of "header" indexes to be used
  // primary purpose for now is to allow for multiple size indexes so type of array doesn't limit length
  arrayIndexFirstValue = static_cast<uint32_t>(rollingArray[0]+0.5);
  arrayMostRecentPositionInsert = static_cast<uint32_t>(rollingArray[1]+0.5);
  sizeInputArrayInsert1 = static_cast<uint32_t>(rollingArray[2]+0.5);
  sizeInputArrayInsert = sizeInputArrayInsert1;

  if (arrayMostRecentPositionInsert == (sizeInputArrayInsert + arrayIndexFirstValue - 1)) // special case for being at the end of the array
  {
    rollingArray[1] = arrayIndexFirstValue;   // update the index that stores most recent value index to the first value index
    rollingArray[arrayIndexFirstValue] = newInputArrayValue;    // write the most recent value to first value index
  }
  else
  {
    rollingArray[1] = arrayMostRecentPositionInsert + 1;
    rollingArray[arrayMostRecentPositionInsert + 1] = newInputArrayValue;
  }
}
void writeToRollingArray(uint32_t rollingArray[], uint32_t newInputArrayValue, uint32_t numberSizeIndex = 1)
{
  // Updated format to allow arbitrary number of "header" indexes to be used
  // primary purpose for now is to allow for multiple size indexes so type of array doesn't limit length
  arrayIndexFirstValue = rollingArray[0];
  arrayMostRecentPositionInsert = rollingArray[1];
  sizeInputArrayInsert1 = rollingArray[2];
  sizeInputArrayInsert = sizeInputArrayInsert1;

  if (arrayMostRecentPositionInsert == (sizeInputArrayInsert + arrayIndexFirstValue - 1)) // special case for being at the end of the array
  {
    rollingArray[1] = arrayIndexFirstValue;   // update the index that stores most recent value index to the first value index
    rollingArray[arrayIndexFirstValue] = newInputArrayValue;    // write the most recent value to first value index
  }
  else
  {
    rollingArray[1] = arrayMostRecentPositionInsert + 1;
    rollingArray[arrayMostRecentPositionInsert + 1] = newInputArrayValue;
  }
}
void writeToRollingArray(uint16_t rollingArray[], uint16_t newInputArrayValue, uint8_t numberSizeIndex = 1)
{
  // Updated format to allow arbitrary number of "header" indexes to be used
  // primary purpose for now is to allow for multiple size indexes so type of array doesn't limit length
  arrayIndexFirstValue = static_cast<uint32_t>(rollingArray[0]);
  arrayMostRecentPositionInsert = static_cast<uint32_t>(rollingArray[1]);
  sizeInputArrayInsert1 = static_cast<uint32_t>(rollingArray[2]);
  sizeInputArrayInsert = sizeInputArrayInsert1;

  if (arrayMostRecentPositionInsert == (sizeInputArrayInsert + arrayIndexFirstValue - 1)) // special case for being at the end of the array
  {
    rollingArray[1] = arrayIndexFirstValue;   // update the index that stores most recent value index to the first value index
    rollingArray[arrayIndexFirstValue] = newInputArrayValue;    // write the most recent value to first value index
  }
  else
  {
    rollingArray[1] = arrayMostRecentPositionInsert + 1;
    rollingArray[arrayMostRecentPositionInsert + 1] = newInputArrayValue;
  }
}
void writeToRollingArray(uint8_t rollingArray[], uint8_t newInputArrayValue, uint8_t numberSizeIndex = 1)
{
  // Updated format to allow arbitrary number of "header" indexes to be used
  // primary purpose for now is to allow for multiple size indexes so type of array doesn't limit length
  arrayIndexFirstValue = static_cast<uint32_t>(rollingArray[0]);
  arrayMostRecentPositionInsert = static_cast<uint32_t>(rollingArray[1]);
  sizeInputArrayInsert1 = static_cast<uint32_t>(rollingArray[2]);
  sizeInputArrayInsert = sizeInputArrayInsert1;

  if (arrayMostRecentPositionInsert == (sizeInputArrayInsert + arrayIndexFirstValue - 1)) // special case for being at the end of the array
  {
    rollingArray[1] = arrayIndexFirstValue;   // update the index that stores most recent value index to the first value index
    rollingArray[arrayIndexFirstValue] = newInputArrayValue;    // write the most recent value to first value index
  }
  else
  {
    rollingArray[1] = arrayMostRecentPositionInsert + 1;
    rollingArray[arrayMostRecentPositionInsert + 1] = newInputArrayValue;
  }
}

void toneBuzzer() //single tone
{

}

void toneBuzzer() //tone program
{

}
