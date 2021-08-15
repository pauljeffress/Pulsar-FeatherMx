/*
 * case_tx_to_CANbus.cpp
 * 
 */

#include "global.h"
#include <CBP_fns.h>
#include <CAN_fns.h>    // xxx - only required here because I call CANloopbackenable() whilst debugging

void case_tx_to_CANbus()
{
    //debugPrint("case_tx_to_CANbus() - executing");

    // if its time to do a routine send (worst case sync method) OR something else has flagged that we should do it then
    if ((CANStatus)  // If the CAN bus is functioning ok and...
            && ((seconds_since_last_CAN_tx > myFeatherMxSettings.FMX_TXCANINT) || (flag_do_CAN_tx)))
    {
        debugPrintln("case_tx_to_CANbus() - ATTEMPTING SEND");
        Serial.println(myCANid);
        delay(1000);     
        //CANloopbackEnable();        // ONLY FOR TESTING!!!!!!!
        
        // Start by sending a CBP_HELLO....just because :)
        CBPsendHello(myCANid);

        CBPsend_uint16_t(myCANid, CBP_FMX_BATT_V, myFeatherMxSettings.FMX_BATT_V);
        CBPsend_int16_t(myCANid, CBP_FMX_TEMP, myFeatherMxSettings.FMX_TEMP);
        CBPsend_uint16_t(myCANid, CBP_FMX_RH, myFeatherMxSettings.FMX_RH);        
        CBPsend_uint32_t(myCANid, CBP_FMX_UPTIME_S, seconds());        

        // Send all of the parameters I have read from the chargers, even if they are still at defaults.
        CBPsend_uint16_t(myCANid, CBP_FMX_PRESS, myFeatherMxSettings.FMX_PRESS);        
        CBPsend_int16_t(myCANid, CBP_FMX_WATERTEMP, myFeatherMxSettings.FMX_WATERTEMP);
        CBPsend_int16_t(myCANid, CBP_FMX_AMBIENTLIGHT, myFeatherMxSettings.FMX_AMBIENTLIGHT);
        
        flag_do_CAN_tx = false; // reset the flag
        seconds_since_last_CAN_tx = 0;  // reset the counter
    }
    else
    {
        //debugPrintln(" - NOT NOW");
    }

    assess_step = check_CANbus; // Set next state
}