/*
 * actuator_fns.cpp
 */

#include "global.h" // My main header file for this project itself


void actuatorsSetup()
{
  debugPrintln("actuatorsSetup() - executing");
  
  /*
   *  Setup the Strobe Light
   */
  debugPrintln("actuatorsSetup() - Strobe Light initialising");
  pinMode(STROBE_LIGHT_PIN, OUTPUT);
  digitalWrite(STROBE_LIGHT_PIN, LOW);  // Start with the Strobe Light off
  // turn Strobe Light on for a few seconds so I can see its working at powerup.
  // Note: its a self flashing strobe, once it has power it just flashes itself at about 1Hz.
  actuatorStrobeOn();
  delay(8000);
  actuatorStrobeOff();


  debugPrintln("actuatorsSetup() - complete");
} // END - actuatorsSetup()

void actuatorStrobeOn()
{
  digitalWrite(STROBE_LIGHT_PIN, HIGH);  // Turn Strobe Light ON 
}

void actuatorStrobeOff()
{
  digitalWrite(STROBE_LIGHT_PIN, LOW);  // Turn Strobe Light OFF 
}
