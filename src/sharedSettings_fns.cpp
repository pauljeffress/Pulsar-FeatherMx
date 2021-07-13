/*
 * SharedSettings_fns.cpp
 */

#include "global.h" // My main header file for this project itself

bool sendSharedSettings_to_AGT(void)
{

    logPrintln("sendSharedSettings_to_AGT() - sending Datum to AGT");

    // Send the Datum to peer
    STdriverF2A.sendDatum(myFeatherSharedSettings);
    seconds_since_last_agt_tx =  0; // reset this counter as we have sent....even though AGT may not have received.

    // Check if its ACKnowledged by the AGT sending back its Datum.
    logPrintln("sendSharedSettings_to_AGT() - waiting for Datum from AGT");
    uint32_t time_limit_millis = 5000;  // 5 seconds - only wait this long
    uint32_t start_time_millis = millis();
    bool success = false;
    while ((!success)  && (millis() < (start_time_millis + time_limit_millis)))     // until we succed or timeout
    {
        if (STdriverF2A.available())    // is there a Datum from the AGT?
        {
            STdriverF2A.rxObj(myAgtSharedSettings); // get the Datum
            logPrintln("sendSharedSettings_to_AGT() - received response Datum from AGT");
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
        logPrintln("sendSharedSettings_to_AGT() - ERROR - did not receive response Datum from AGT!");
    }
    return(success);
}   // END - sendSharedSettings_to_AGT


void initFeatherSharedSettings(void) // Initialises the SharedSettings in RAM with the default values
{
    // initialise based on the main myFeatherSettings, they are the source of truth.
  myFeatherSharedSettings.BATTV = myFeatherSettings.BATTV;
  myFeatherSharedSettings.PRESS = myFeatherSettings.PRESS;
  myFeatherSharedSettings.AIRTEMP = myFeatherSettings.AIRTEMP;
  myFeatherSharedSettings.HUMID = myFeatherSettings.HUMID;
  myFeatherSharedSettings.WATERTEMP = myFeatherSettings.WATERTEMP;
  myFeatherSharedSettings.AMBIENTLIGHT = myFeatherSettings.AMBIENTLIGHT;
  
  myFeatherSharedSettings.YEAR = myFeatherSettings.YEAR;
  myFeatherSharedSettings.MONTH = myFeatherSettings.MONTH;
  myFeatherSharedSettings.DAY = myFeatherSettings.DAY;
  myFeatherSharedSettings.HOUR = myFeatherSettings.HOUR;
  myFeatherSharedSettings.MIN = myFeatherSettings.MIN;
  myFeatherSharedSettings.MILLIS = myFeatherSettings.MILLIS;

  myFeatherSharedSettings.GPSTIMESTAMP = myFeatherSettings.GPSTIMESTAMP;
  myFeatherSharedSettings.LAT = myFeatherSettings.LAT;
  myFeatherSharedSettings.LON = myFeatherSettings.LON;
  myFeatherSharedSettings.ALT = myFeatherSettings.ALT;
  myFeatherSharedSettings.SPEED = myFeatherSettings.SPEED;
  myFeatherSharedSettings.HEAD = myFeatherSettings.HEAD;
  myFeatherSharedSettings.SATS = myFeatherSettings.SATS;
  myFeatherSharedSettings.PDOP = myFeatherSettings.PDOP;
  myFeatherSharedSettings.FIX = myFeatherSettings.FIX;

  debugPrintln("initFeatherSharedSettings: RAM settings initialised");
}   // END - initSharedSettings()