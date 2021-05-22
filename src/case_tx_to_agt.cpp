/*
 * case_tx_to_agt.cpp
 * 
 */

#include "global.h"

void case_tx_to_agt()
{
    //debugPrintln("case_tx_to_agt() - executing");

    // Send Dummy Datum
        // Because we only allow the AGT to TX to us when we are in our case_rx_from_AGT() state, we don't
        // need to check/set any flags to prevent the Feather from simultaneously TX'ing to us here.  
        // If we want to send to the AGT here, we just send.
        if (millis() > (lastsend + (1000 * SENDPERIODSECONDS))) // is it time to send another dummy packet?
        {
            lastsend = millis(); // timestamp this send.
            //ledInitFlasher();

            // insert our dummy data
            STDatumTX.i1++;
            if (STDatumTX.i1 > 250) STDatumTX.i1 = 1;   // roll it over every 250 
            //    STDatumTX.f += 1.1;

            Serial.println("case_tx_to_agt() - Sending a Datum");
            Serial.println("case_tx_to_agt() - SSSSSSSSSSSSSSS");
            Serial.print("case_tx_to_agt() - i1=");
            Serial.println(STDatumTX.i1);
            Serial.print("case_tx_to_agt() - c1=");
            Serial.println(STDatumTX.c1);
            Serial.print("case_tx_to_agt() - c2=");
            Serial.println(STDatumTX.c2);
            Serial.print("case_tx_to_agt() - c3=");
            Serial.println(STDatumTX.c3);
            Serial.print("case_tx_to_agt() - c4=");
            Serial.println(STDatumTX.c4);

            // Send the Datum to peer
            STdriverF2A.sendDatum(STDatumTX);

        } // END of send dummy packet


    assess_step = tx_to_logger; // Set next state
}