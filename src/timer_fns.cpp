/*
 * timer_fns.cpp
 *
 * My routines that use the SAMD_TimerInterrupt Library - https://github.com/khoih-prog/SAMD_TimerInterrupt
 * Based on the libraries example code in "TimerInterruptLEDDemo.ino"
 */

#include "global.h"        // My main header file for this project itself

// void TimerHandler0()
// {
//   mytimercounter++;
// }

void timerSetup(void)
{
  // Gets called from case_loop_init as we need to I am following the same model in the code as I have in the
  // AGT, where it accomodates the MCU going to sleep and certain things needing to be setup post sleep
  // rather than in setup() which only gets called post HW reset.

  debugPrintln("timerSetup() - executing");

  seconds_since_reset_or_powercycle = 0;
  seconds_since_last_wake = 0;
  seconds_since_last_ap_tx = 0;
  seconds_since_last_ap_rx = 0;
  seconds_since_last_agt_tx = 0;
  seconds_since_last_agt_rx = 0;
  seconds_since_last_sensors_read = 0;
  
  // Proper code below to reinstate when I fix Linker error
  // // Interval in microsecs
  // if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
  // {
  //   Serial.println("Timer set");
  // }
  // else
  //   Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
}

void timerIncrementer(void)
{
  //debugPrint("timerIncrementer() - executing at ");
  //Serial.print(millis());
  seconds_since_reset_or_powercycle++;
  seconds_since_last_wake++;
  seconds_since_last_ap_tx++;
  seconds_since_last_ap_rx++;
  seconds_since_last_agt_tx++;
  seconds_since_last_agt_rx++;
  seconds_since_last_sensors_read++;
  //debugPrint(" seconds_since_last_wake=");
  //Serial.println(seconds_since_last_wake);
 }