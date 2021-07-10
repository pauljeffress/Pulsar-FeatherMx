/*
 * case_rx_from_agt.cpp
 * 
 */

#include "global.h"

void case_rx_from_agt()
{
    //debugPrintln("case_rx_from_agt() - executing");
   
    if (STdriverF2A.available())
    {
        logPrintln("case_rx_to_agt() - receiving Dummy Datum from AGT");
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

    assess_step = tx_to_agt; // we can skip "process_agt" as we did not receive anything from it.

}   // END - case_rx_from_agt()