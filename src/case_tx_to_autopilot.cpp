/*
 * case_tx_to_autopilot.cpp
 * 
 */

#include "global.h"

/*============================
 * case_tx_to_autopilot()
 *
 * Do a periodic TX to the AP, via MAVLink messages. 
 * AND/OR
 * Do a specifically flagged TX to the AP.
 * 
 *============================*/
void case_tx_to_autopilot()
{
    // if its time to do a periodic TX to the AutoPilot?
    if (seconds_since_last_ap_tx > TX_TO_AP_PERIOD_SECONDS)
    {
        #ifdef MAVLINK_DEBUG 
            debugPrintln("case_tx_to_autopilot() - Periodic TX - Time to execute.");
        #endif

        seconds_since_last_ap_tx = 0;   // reset timer.

        // Do any periodic TX to AP jobs here.....

        #ifdef MAVLINK_DEBUG 
            debugPrintln("case_tx_to_autopilot() - Periodic TX - calling set_disarm_ap()");
        #endif

        // set_one_param_from_ap();     // only for testing the initial capability
        // set_arm_ap();       // only for testing the initial capability
        // set_disarm_ap();    // only for testing the initial capability
        // set_flightmode_ap(ROVER_MODE_STEERING);


        #ifdef MAVLINK_DEBUG 
            debugPrintln("case_tx_to_autopilot() - Periodic TX - do immediate mavlink_receive() to check for response");
        #endif

        // do an immediate mavlink_receive() to see if the AP responded to me setting the param above.
        uint32_t start = millis();  // xxx - need to review how I'm timing this loop...seems clunky. Also need to take any contants and set the as #defines.
        while ((millis() < (start + 3000)) && (millis() > 4000))  // For 3 seconds, see if we can assemble received msgs and if so process them.
            mavlink_receive(); 

        #ifdef MAVLINK_DEBUG 
            debugPrintln("case_tx_to_autopilot() - Periodic TX - Finished.");
        #endif

    }
    
    // do we need to do a specifically flagged TX to the AutoPilot?

    // if (special flag)
    // {
    //     #ifdef MAVLINK_DEBUG 
    //         debugPrintln("case_rt_from_autopilot() - Specially flagged TX - execute.");
    //     #endif

    //     special flag = false = 0;   // reset the flag.
    // }
    
}   // END - case_tx_to_autopilot()