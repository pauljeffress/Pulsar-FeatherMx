/*
 * SharedSettings_fns.cpp
 *
 * This file may be similar on both the Feather and AGT but is slighhtly custom per side.
 */

#include "global.h" // My main header file for this project itself

bool sendSharedSettings_to_AGT(void)
{

    debugPrintln("sendSharedSettings_to_AGT() - sending Datum to AGT -------------------------------------------");

    bool success = false;

    preptosendFeatherSharedSettings();    // gets myFeatherSharedSettings fresh and ready to TX to AGT. 

    // Send the Datum to peer
    int8_t numbytessent = STdriverF2A.sendDatum(myFeatherSharedSettings);
    //debugPrint("sendSharedSettings_to_AGT() - sending Datum to AGT - numbytessent:");
    //debugPrintlnInt(numbytessent);
    seconds_since_last_agt_tx =  0; // reset this counter as we have sent....even though AGT may not have received.

    return(success);
}   // END - sendSharedSettings_to_AGT


void initFeatherSharedSettings(void) // Initialises the myFeatherSharedSettings in RAM with the default values
{
  // myFeatherSharedSettings.MAGICNUM = 0;   // do not reset this, the SharedSetting is the source of truth!!!!
  // initialise based on the main myFeatherSettings, they are the source of truth.
  myFeatherSharedSettings.BATTV = myFeatherSettings.BATTV;
  myFeatherSharedSettings.PRESS = myFeatherSettings.PRESS;
  myFeatherSharedSettings.AIRTEMP = myFeatherSettings.AIRTEMP;
  myFeatherSharedSettings.HUMID = myFeatherSettings.HUMID;
  myFeatherSharedSettings.WATERTEMP = myFeatherSettings.WATERTEMP;
  myFeatherSharedSettings.AMBIENTLIGHT = myFeatherSettings.AMBIENTLIGHT;
  
  myFeatherSharedSettings.GPSYEAR = myFeatherSettings.GPSYEAR;
  myFeatherSharedSettings.GPSMONTH = myFeatherSettings.GPSMONTH;
  myFeatherSharedSettings.GPSDAY = myFeatherSettings.GPSDAY;
  myFeatherSharedSettings.GPSHOUR = myFeatherSettings.GPSHOUR;
  myFeatherSharedSettings.GPSMIN = myFeatherSettings.GPSMIN;
  myFeatherSharedSettings.GPSSEC = myFeatherSettings.GPSSEC;
  myFeatherSharedSettings.GPSMILLIS = myFeatherSettings.GPSMILLIS;

  myFeatherSharedSettings.LAT = myFeatherSettings.LAT;
  myFeatherSharedSettings.LON = myFeatherSettings.LON;
  myFeatherSharedSettings.ALT = myFeatherSettings.ALT;
  myFeatherSharedSettings.SPEED = myFeatherSettings.SPEED;
  myFeatherSharedSettings.HEAD = myFeatherSettings.HEAD;
  myFeatherSharedSettings.SATS = myFeatherSettings.SATS;
  myFeatherSharedSettings.PDOP = myFeatherSettings.PDOP;
  myFeatherSharedSettings.FIX = myFeatherSettings.FIX;

  //debugPrintln("initFeatherSharedSettings: RAM settings initialised");
}   // END - initFeatherSharedSettings()


void preptosendFeatherSharedSettings(void) // gets myFeatherSharedSettings fresh (mainly from myFeatherSettings) and ready to TX to AGT.
{
  initFeatherSharedSettings(); // start by setting everything to align with the current myFeatherSettings
  
  myFeatherSharedSettings.MAGICNUM++;
  if (myFeatherSharedSettings.MAGICNUM > 240) // roll the counter back to 0 at 240.
    myFeatherSharedSettings.MAGICNUM = 0;
  // then alter anything that needs to change just before transmission to the AGT.
  // xxx TBD

  //debugPrintln("preptosendFeatherSharedSettings: DONE");
}   // END - preptosendFeatherSharedSettings()




