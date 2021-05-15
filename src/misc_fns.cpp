/*
 * misc_fns.ino
 * 
 * Assorted functions
 * 
 * 
 */

#include "global.h"

void setupPins()
{
  debugPrintln("setupPins() - CODE STILL TO BE DONE IN HERE!!!!!!!!!");

  pinMode(LED_BUILTIN, OUTPUT); // Make the LED pin an output

  // pinMode(geofencePin, INPUT); // Configure the geofence pin as an input

  // pinMode(iridiumPwrEN, OUTPUT); // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  // digitalWrite(iridiumPwrEN, LOW); // Disable Iridium Power (HIGH = enable; LOW = disable)

  // pinMode(superCapChgEN, OUTPUT); // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  // digitalWrite(superCapChgEN, LOW); // Disable the super capacitor charger (HIGH = enable; LOW = disable)

  // pinMode(iridiumSleep, OUTPUT); // Iridium 9603N On/Off (Sleep) pin
  // digitalWrite(iridiumSleep, LOW); // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)

  // pinMode(iridiumRI, INPUT); // Configure the Iridium Ring Indicator as an input
  // pinMode(iridiumNA, INPUT); // Configure the Iridium Network Available as an input

  // pinMode(superCapPGOOD, INPUT); // Configure the super capacitor charger PGOOD input

  // pinMode(busVoltageMonEN, OUTPUT); // Make the Bus Voltage Monitor Enable an output
  // digitalWrite(busVoltageMonEN, LOW); // Set it low to disable the measurement to save power

  // // control pins for serial connection to Feather
  // pinMode(FEATHER_READY_TO_RX_PIN, INPUT);
  // pinMode(AGT_WANTS_TO_TX_PIN, OUTPUT);
  // digitalWrite(AGT_WANTS_TO_TX_PIN, LOW); // set initial state
}

// void AGT_WANTS_TO_TX_PINhigh(){
//   digitalWrite(AGT_WANTS_TO_TX_PIN, HIGH);
//   digitalWrite(LED_BUILTIN, HIGH);
// }

// void AGT_WANTS_TO_TX_PINlow(){
//   digitalWrite(AGT_WANTS_TO_TX_PIN, LOW);
//   digitalWrite(LED_BUILTIN, LOW);
// }
