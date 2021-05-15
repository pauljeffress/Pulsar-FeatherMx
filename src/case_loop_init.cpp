/*
 * case_loop_init.cpp
 * 
 * Code to execute when doing loop_init
 * 
 * 
 */

#include "global.h"

// ************************************************************************************************
// Initialise things, gets called either first time through loop() OR after wake()

void case_loop_init()
{
  // Start the console serial port and send the welcome message
  Serial.begin(115200);
  delay(2000); // ensure time for the Serial port to get ready.

  enableDebugging(Serial); // THIS LINE IS RIGHT HERE FOR A REASON. Because we re issue Serial.begin() for the console/debug port
                           // just a few lines earlier here in case_loop_init(), you need to enable the debug stuff that uses it
                           // after you have done that.  For more info see case_zzz(), and you will see it expklicitly does a
                           // Serial.end(); before putting the processor to sleep.
                           // Uncomment this line to enable extra debug messages to Serial

  debugPrintln("================================================");
  debugPrintln("************************************************");
  debugPrintln("================================================");
  debugPrintln("case_loop_init() - Pulsar FeatherMx (Re)starting");

  // do any other general setup tasks that would normally be done in setup() but need to
  // be done here due to sleep/wake.
  tftSetup();
  sensorsSetup();

  loop_step = assess_situation; // Set next state
  assess_step = check_power;    // Ensure assess_situation() starts at correct first step.
}
