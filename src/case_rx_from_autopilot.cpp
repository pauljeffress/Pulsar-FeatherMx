/*
 * case_rx_from_autopilot.cpp
 * 
 */

#include "global.h"

void case_rx_from_autopilot()
{
    // I need to ensure that the mavlink_receive() function gets a solid couple of contiguous
    // seconds to ensure it gets a good chance to snap up lots of mavlink data and form packets.
    // If I give it just a few milliseconds each time its not enough as it seems that the other
    // states in assess_situation() take up too much time and I think that serial chars from 
    // the autopilot were getting dropped and so I wasn't getting full packets reliably.
    // The other key thing to consider here is that the mavlink_request_datastream() function
    // has asked the autopilot to send data once a second, so waiting here for a few seconds gives
    // me multiple chances to chatch what I need in the mavlink stream.
    // Also, as I am concerned about this while loop playing up when millis() rolls over to zero, 
    // I am using the extra "&& (millis() > 4000)" in the while() to jump that bit close to 0.
    
    //debugPrintln("case_rx_from_autopilot() - executing");
    
    // if its time to do a routine read of AutoPilot MAVlink data?
    if (seconds_since_last_ap_rx > myFeatherSettings.RXAPINT)
    {
        //debugPrintln(" - ATTEMPTING RX");
        uint32_t start = millis();  // xxx - need to review how I'm timing this loop...seems clunky. Also need to take any contants and set the as #defines.
        while ((millis() < (start + 3000)) && (millis() > 4000))  // keep doing it for 3 seconds
            mavlink_receive(); 
        seconds_since_last_ap_rx = 0;    // reset counter
    }
    else
    {
        //debugPrintln(" - NOT NOW");
    }

    //debugPrintln("case_rx_from_autopilot() - done");

    assess_step = process_autopilot; // Set next state
}