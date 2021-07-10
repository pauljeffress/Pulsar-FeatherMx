/*
 * debug_fns.ino
 * 
 * Debugging specific functions
 * 
 * 
 */

#include "global.h"

// debug functions taken from the SparkFun u-blox library:
// https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library


void enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging

  _printDebug = true; //Should we print the commands we send? Good for debugging
}

void disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

void debugPrint(const char *message)
//Safely print messages
{
  if (_printDebug == true)
  {
    _debugSerial->print(message);
  }
}

void debugPrintln(const char *message)
//Safely print messages
{
  if (_printDebug == true)
  {
    _debugSerial->println(message);
  }
}

void debugPrintInt(int32_t number)
//Safely print a number
{
  if (_printDebug == true)
  {
    _debugSerial->print(number);
  }
}

void debugPrintlnInt(int32_t number)
//Safely print a number
{
  if (_printDebug == true)
  {
    _debugSerial->println(number);
  }
}

void debugPrintFlt(float number)
//Safely print a number
{
  if (_printDebug == true)
  {
    _debugSerial->print(number);
  }
}

void debugPrintlnFlt(float number)
//Safely print a number
{
  if (_printDebug == true)
  {
    _debugSerial->println(number);
  }
}