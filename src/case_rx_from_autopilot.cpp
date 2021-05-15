/*
 * case_rx_from_autopilot.cpp
 * 
 */

#include "global.h"

void case_rx_from_autopilot()
{
    //debugPrintln("case_rx_from_autopilot() - executing");

    mavlink_receive();  // 


    assess_step = process_autopilot; // Set next state
}