/*
 * case_tx_to_agt.cpp
 * 
 */

#include "global.h"

void case_tx_to_agt()
{
    //debugPrint("case_tx_to_agt() - executing");

    // if its time to do a routine send (worst case sync method) OR something else has flagged that we should do it then
    if ((seconds_since_last_agt_tx > myFeatherMxSettings.FMX_TXAGTINT) or (flag_do_agt_tx))
    {
        //debugPrintln(" - ATTEMPTING SEND");
        sendSharedSettings_to_AGT();
        flag_do_agt_tx = false;
    }
    else
    {
        //debugPrintln(" - NOT NOW");
    }

    assess_step = tx_to_logger; // Set next state
}