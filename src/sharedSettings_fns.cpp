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
  // initialise based on the main myFeatherMxSettings, they are the source of truth.
  myFeatherSharedSettings.BATTV = myFeatherMxSettings.FMX_BATT_V;
  myFeatherSharedSettings.PRESS = myFeatherMxSettings.FMX_PRESS;
  myFeatherSharedSettings.TEMP = myFeatherMxSettings.FMX_TEMP;
  myFeatherSharedSettings.HUMID = myFeatherMxSettings.FMX_RH;
  myFeatherSharedSettings.WATERTEMP = myFeatherMxSettings.FMX_WATERTEMP;
  myFeatherSharedSettings.AMBIENTLIGHT = myFeatherMxSettings.FMX_AMBIENTLIGHT;
  
  myFeatherSharedSettings.GPSYEAR = myFeatherMxSettings.FMX_GPSYEAR;
  myFeatherSharedSettings.GPSMONTH = myFeatherMxSettings.FMX_GPSMONTH;
  myFeatherSharedSettings.GPSDAY = myFeatherMxSettings.FMX_GPSDAY;
  myFeatherSharedSettings.GPSHOUR = myFeatherMxSettings.FMX_GPSHOUR;
  myFeatherSharedSettings.GPSMIN = myFeatherMxSettings.FMX_GPSMIN;
  myFeatherSharedSettings.GPSSEC = myFeatherMxSettings.FMX_GPSSEC;
  myFeatherSharedSettings.GPSMILLIS = myFeatherMxSettings.FMX_GPSMILLIS;

  myFeatherSharedSettings.LAT = myFeatherMxSettings.FMX_LAT;
  myFeatherSharedSettings.LON = myFeatherMxSettings.FMX_LON;
  myFeatherSharedSettings.ALT = myFeatherMxSettings.FMX_ALT;
  myFeatherSharedSettings.SPEED = myFeatherMxSettings.FMX_SPEED;
  myFeatherSharedSettings.HEAD = myFeatherMxSettings.FMX_HEAD;
  myFeatherSharedSettings.SATS = myFeatherMxSettings.FMX_SATS;
  myFeatherSharedSettings.PDOP = myFeatherMxSettings.FMX_PDOP;
  myFeatherSharedSettings.FIX = myFeatherMxSettings.FMX_FIX;

  //debugPrintln("initFeatherSharedSettings: RAM settings initialised");
}   // END - initFeatherSharedSettings()


void preptosendFeatherSharedSettings(void) // gets myFeatherSharedSettings fresh (mainly from myFeatherMxSettings) and ready to TX to AGT.
{
  initFeatherSharedSettings(); // start by setting everything to align with the current myFeatherMxSettings
  
  myFeatherSharedSettings.MAGICNUM++;
  if (myFeatherSharedSettings.MAGICNUM > 240) // roll the counter back to 0 at 240.
    myFeatherSharedSettings.MAGICNUM = 0;
  // then alter anything that needs to change just before transmission to the AGT.
  // xxx TBD

  //debugPrintln("preptosendFeatherSharedSettings: DONE");
}   // END - preptosendFeatherSharedSettings()




