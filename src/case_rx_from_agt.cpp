/*
 * case_rx_from_agt.cpp
 * 
 */

#include "global.h"

void case_rx_from_agt()
{
    bool agt_wants_to_tx_to_us = false;
    uint32_t waitmSec = 0;

    //debugPrintln("case_rx_from_agt() - executing");

    setFEATHER_READY_TO_RX_PINlow();

    // Because we only ever TX to the AGT when we are in our case_tx_to_AGT() state, we don't
    // need to check/set any flags to prevent ourselves from simultaneously TX'ing while RX'ing here.  
    // If the AGT is indicating it wants to send us something, we just let let it.

    // Is the AGT signalling it wants to TX to me?
    agt_wants_to_tx_to_us = digitalRead(AGT_WANTS_TO_TX_PIN);
    if (agt_wants_to_tx_to_us)  // If it is, then respond and let it know it can, and block myself from TX'in until its done.
    { 
        // respond by Asserting FEATHER_READY_TO_RX_PIN
        Serial.println("case_rx_from_agt() - AGT signalling it wants to TX to us");
        setFEATHER_READY_TO_RX_PINhigh();
        //Serial.println("case_rx_from_agt() - Asserting FEATHER_READY_TO_RX_PIN to let AGT know it can safely TX");

        // wait here until the AGT has finished and/or no longer wants to TX to us (by dropping its AGT_WANTS_TO_TX_PIN) OR a timeout occurs
        waitmSec = FEATHER_WAIT_FOR_AGT_TO_DROP * 1000; // load the timer in mSecs
        while (digitalRead(AGT_WANTS_TO_TX_PIN) && ( waitmSec > 0))
        {
            // while this while() is executing the Serial ISR is hopefully receiving chars from the AGT serial port.
            delay(10);  // wait for 10mS before checking if Feather has dropped FEATHER_READY_TO_RX_PIN again.
            waitmSec = waitmSec - 10;  // take 10mSec off the timer
        }
        //Serial.print("case_rx_from_agt() - waitmSec = "); Serial.println(waitmSec);

        // by this point we have either given the AGT enough time to send stuff to us OR we've hit the timeout, either
        // way its time to clean up signalling PINs on our side and move on.  The AGT needs to independently take care of 
        // the PIN cleanup on its side.
        setFEATHER_READY_TO_RX_PINlow();
        //Serial.println("case_rx_from_agt() - Dropping FEATHER_READY_TO_RX_PIN to cleanup and let AGT know it can NO LONGER safely TX");
    
        // Check if the AGT has sent us anything in the perod we gave it to try.
        // Use SerialTransfer receive code to get anything from our peer.
        
        //while (Serial2.available()) 
        //{        
        //    Serial.print("Serial2 RX 0x"); Serial.println(Serial2.read(), HEX);
        //}
        
        if (STdriverF2A.available())
        {
            STdriverF2A.rxObj(STDatumRX);
            Serial.println("case_rx_from_agt() - Received a Datum from AGT");
            Serial.println("case_rx_from_agt() - RRRRRRRRRRRRRRRRRRRRRRRRR");
            Serial.print("case_rx_from_agt() - i1=");
            Serial.println(STDatumRX.i1);
            Serial.print("case_rx_from_agt() - c1=");
            Serial.println(STDatumRX.c1);
            Serial.print("case_rx_from_agt() - c2=");
            Serial.println(STDatumRX.c2);
            Serial.print("case_rx_from_agt() - c3=");
            Serial.println(STDatumRX.c3);
            Serial.print("case_rx_from_agt() - c4=");
            Serial.println(STDatumRX.c4);

            assess_step = process_agt; // now that we have data from the AGT lets process it
        }
        else
        {
            Serial.println("case_rx_from_agt() - WARNING - ST datum from AGT expected but not received");
        }
    }  // END - if (agt_wants_to_tx_to_us)
    assess_step = tx_to_agt; // we can skip "process_agt" as we did not receive anything from it.
}   // END - case_rx_from_agt()