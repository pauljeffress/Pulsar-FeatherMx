/*
 * case_assess_situation.ino
 * 
 */

#include "global.h"

void case_assess_situation()
{
// We will get run every WAKEINT and we stay in here running our state machine multiple times
// until we need to initiate SLEEP, at which point we exit and the state machine in loop() will 
// take us to global SLEEP state.
// We are only ever called from the main state machine's case_loop_init()
//
// Keep stepping through this sub state machine until we decide either we have
// done all we need to do and we can now go back to SLEEP or if we need to go to SLEEP 
// ASAP due to say low power. Once we know power is very low and hence any further actions are a risk,
// there is no point staying awake. 

//  STATES
//  ------
//  check_power
//  read_sensors
//  write_to_tft
//  rx_from_autopilot
//  process_autopilot
//  tx_to_autopilot
//  rx_from_agt
//  process_agt
//  tx_to_agt
//  tx_to_logger
//  tickle_watchdog
//  sleep_yet

// debugPrintln("case_assess_situation() - starting");

flag_do_agt_tx = false;   // when set, tx_to_agt() will send the full standard 
                        // Feather to AGT  blob of data to the AGT, and 
                        // the AGT will decide what to do with it.



iterationCounter = iterationCounter + 1; // Increment the iterationCounter, helps detect CPU RESET or Power Cycle.

while (loop_step == assess_situation)   // Stay in this state machine or is it time to exit out to the main loop state machine?
  {
  //debugPrintln("case_assess_situation() - top of main while loop");

  // Do LED
  digitalWrite( LED_BUILTIN, millis() % 500 > 250 );

  // temporarily required intil my 1sec Timer ISR is working
  // determines if 1sec has passed (roughly) and if so increments my seconds counters
  oneSecCounter = millis();
  if(oneSecCounter > (oneSecCounter_last + 1000))
  {
    unsigned long numSecsPassed = (oneSecCounter - oneSecCounter_last) / 1000;   // how many whole seconds have passed
    oneSecCounter_last = oneSecCounter;
    for(unsigned long i=0; i < numSecsPassed; i++)  // call my incrementer once for each second that has passed.
      timerIncrementer();
  }
  
  switch (assess_step) 
    {
    // ************************************************************************************************
    // Check Feather's power source status.
    case check_power:
      case_check_power();  
      assess_step = read_sensors;  // Set next state here because we can't set it inside the function itself as its a shared function. 
    break;

    // ************************************************************************************************
    // Read the various sensors attached to the Feather.
    case read_sensors:
      case_read_sensors(); 
      //assess_step = write_to_tft; 
      //debugPrintln("Skipping read_sensors");   
    break;

    // ************************************************************************************************
    // If we need to, send info onto the CAN bus
    case tx_to_CANbus:
      case_tx_to_CANbus();    
    break;
    
    // ************************************************************************************************
    // check if any packets received via CAN bus.
    case check_CANbus:
      case_check_CANbus(); 
      assess_step = write_to_tft; // Set next state here because we can't set it inside the function itself as its a shared function.
    break;
    
    // ************************************************************************************************
    // Update the tft display
    case write_to_tft:
      //Serial.println("case_write_to_tft()===============================");
      case_write_to_tft(); 
      //assess_step = rx_from_agt;  
      //debugPrintln("Skipping write_to_tft"); 
    break;

    // ************************************************************************************************
    // Read Mavlink stream from the Autopilot.
    case rx_from_autopilot:
      //Serial.println("case_rx_from_autopilot()+++++++++++++++++++++++++++");
      case_rx_from_autopilot();
      //assess_step = check_power;  // temorary short cut back to start of state machine.  
      //delay(500);  
    break;

    // ************************************************************************************************
    // Review/action the recently received Mavlink data from the Autopilot.
    case process_autopilot: 
      case_process_autopilot();    
    break;
      
    // ************************************************************************************************
    // Send Mavlink data to the autopilot.
    case tx_to_autopilot:
      case_tx_to_autopilot();
    break;

    // ************************************************************************************************
    // Check if the AGT has sent us a datum
    case rx_from_agt:
      case_rx_from_agt();     
    break;

    // ************************************************************************************************
    // process it if it has and set appropriate flags
    case process_agt:
      case_process_agt();    
    break;

    // ************************************************************************************************
    // If we need to, send a datum to the AGT
    case tx_to_agt:
      case_tx_to_agt();    
    break;

    // ************************************************************************************************
    // Decide and write to the Logger.
    case tx_to_logger: 
      case_tx_to_logger();    
    break;
      
    // ************************************************************************************************
    // Tickle the watchdog so it knows we are ok.
    case tickle_watchdog:
      case_tickle_watchdog();
    break;

    // ************************************************************************************************
    // Look at flags and timers/counter, decide if should we be going to SLEEP?
    case sleep_yet:
      case_sleep_yet();
    break;

    // ************************************************************************************************
    // DEFAULT - should not happen, but programing it defensively
    default:
      debugPrintln("ERROR - we hit the default: in assess_situation state machine");
      loop_step = loop_init;  // go to loop_init to try and clean up from whatever caused this!
    break;    

    } // End of switch (assess_step)
    //debugPrintln("case_assess_situation() - bottom of main while loop");
  } // End of while(loop_step == assess_situation)
  debugPrintln("case_assess_situation() finishing");
} // End of case_assess_situation()
