/*
 * case_check_power.cpp
 * 
 */

#include "global.h"

void case_check_power()
{
  //debugPrintln("case_check_power() executing");

  // measure it as per https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51/power-management
  float measuredvbat = analogRead(FEATHER_VBAT_PIN);
  measuredvbat *= 2;    // we divided by 2 with the resistor divider circuit, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  //debugPrint("case_check_power() - Feather Vbat: ");
  //debugPrintlnFlt(measuredvbat);

  if (measuredvbat < FEATHER_VBAT_LOW)
  {
    debugPrintln("case_check_power() - Battery low, so forcing SLEEP");
    loop_step = zzz; // Battery is low, force my way out to global SLEEP functions.
  }
  else
    assess_step = read_sensors; // Set next state
}
