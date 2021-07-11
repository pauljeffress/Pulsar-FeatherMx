/*
 * sharedSettings_fns.cpp
 */

#include "global.h" // My main header file for this project itself

bool send_sharedSettings_to_AGT(void)
{

    logPrintln("send_sharedSettings_to_AGT() - sending Datum to AGT");

    // Send the Datum to peer
    STdriverF2A.sendDatum(myfeathersharedSettings);
    seconds_since_last_agt_tx =  0; // reset this counter as we have sent....even though AGT may not have received.

    // Check if its ACKnowledged by the AGT sending back its Datum.
    logPrintln("send_sharedSettings_to_AGT() - waiting for Datum from AGT");
    uint32_t time_limit_millis = 5000;  // 5 seconds - only wait this long
    uint32_t start_time_millis = millis();
    bool success = false;
    while ((!success)  && (millis() < (start_time_millis + time_limit_millis)))     // until we succed or timeout
    {
        if (STdriverF2A.available())    // is there a Datum from the AGT?
        {
            STdriverF2A.rxObj(myagtsharedSettings); // get the Datum
            logPrintln("send_sharedSettings_to_AGT() - received response Datum from AGT");
            seconds_since_last_agt_rx = 0; // reset this counter as we have just received from the AGT.
            success = true;
        }
    }

    if (success)    // We did get a Datum, so we need to take action
    {
        // xxx - add code to either flag or just start processing the received 
    }
    else
    {
        logPrintln("send_sharedSettings_to_AGT() - ERROR - did not receive response Datum from AGT!");
    }

}   // END - send_sharedSettings_to_AGT


