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
    debugPrint("sendSharedSettings_to_AGT() - sending Datum to AGT - numbytessent:");
    debugPrintlnInt(numbytessent);
    seconds_since_last_agt_tx =  0; // reset this counter as we have sent....even though AGT may not have received.

    return(success);
}   // END - sendSharedSettings_to_AGT


void initFeatherSharedSettings(void) // Initialises the myFeatherSharedSettings in RAM & is also called
                                     // to update myFeatherSharedSettings just before we do a send 
                                     // over to the AGT.
{
  // myFeatherSharedSettings.MAGICNUM = 0;   // do not reset this, the SharedSetting is the source of truth!!!!
  
  // initialise based on the correct source of truth. This is the same for both initial boot up AND when we need to refresh!
  myFeatherSharedSettings.PF_BATT_V = myPowerFeatherSettings.PF_BATT_V;
  myFeatherSharedSettings.PF_TEMP = myPowerFeatherSettings.PF_TEMP;
  myFeatherSharedSettings.PF_RH = myPowerFeatherSettings.PF_RH;
  myFeatherSharedSettings.PF_UPTIME_S = myPowerFeatherSettings.PF_UPTIME_S;
  
  myFeatherSharedSettings.PF_BATT1_SOC = myPowerFeatherSettings.PF_BATT1_SOC;
  myFeatherSharedSettings.PF_BATT1_V = myPowerFeatherSettings.PF_BATT1_V;
  myFeatherSharedSettings.PF_BATT1_CHARGE_I = myPowerFeatherSettings.PF_BATT1_CHARGE_I;

  myFeatherSharedSettings.PF_BATT2_SOC = myPowerFeatherSettings.PF_BATT2_SOC;
  myFeatherSharedSettings.PF_BATT2_V = myPowerFeatherSettings.PF_BATT2_V;
  myFeatherSharedSettings.PF_BATT2_CHARGE_I = myPowerFeatherSettings.PF_BATT2_CHARGE_I;

  myFeatherSharedSettings.AP_GPSTIMESTAMP = myFeatherMxSettings.AP_GPSTIMESTAMP;
  myFeatherSharedSettings.AP_LAT = myFeatherMxSettings.AP_LAT;
  myFeatherSharedSettings.AP_LON = myFeatherMxSettings.AP_LON;
  myFeatherSharedSettings.AP_SPEED = myFeatherMxSettings.AP_SPEED;
  myFeatherSharedSettings.AP_COG = myFeatherMxSettings.AP_COG;
  myFeatherSharedSettings.AP_SATS = myFeatherMxSettings.AP_SATS;
  myFeatherSharedSettings.AP_FIX = myFeatherMxSettings.AP_FIX;

  myFeatherSharedSettings.AP_CUSTOMMODE = myFeatherMxSettings.AP_CUSTOMMODE;
  myFeatherSharedSettings.AP_SYSTEMSTATUS = myFeatherMxSettings.AP_SYSTEMSTATUS;

  myFeatherSharedSettings.FMX_BATT_V = myFeatherMxSettings.FMX_BATT_V;
  myFeatherSharedSettings.FMX_TEMP = myFeatherMxSettings.FMX_TEMP;
  myFeatherSharedSettings.FMX_RH = myFeatherMxSettings.FMX_RH;
  myFeatherSharedSettings.FMX_WATERTEMP = myFeatherMxSettings.FMX_WATERTEMP;
  myFeatherSharedSettings.FMX_AMBIENTLIGHT = myFeatherMxSettings.FMX_AMBIENTLIGHT;
  myFeatherSharedSettings.FMX_UPTIME_S = seconds(); // xxx - should this be coming from myFeatherMxSettings.FMX_UPTIME_S ???

  //debugPrintln("initFeatherSharedSettings: RAM settings initialised");
}   // END - initFeatherSharedSettings()


void preptosendFeatherSharedSettings(void) // gets myFeatherSharedSettings fresh (mainly from myFeatherMxSettings) and ready to TX to AGT.
{
  // refresh all parameters in myFeatherSharedSettings.
  initFeatherSharedSettings();

  // update the MAGICNUM in myFeatherSharedSettings.
  myFeatherSharedSettings.FMX_MAGICNUM++;
  if (myFeatherSharedSettings.FMX_MAGICNUM > 240) // roll the counter back to 0 at 240.
    myFeatherSharedSettings.FMX_MAGICNUM = 0;

  //debugPrintln("preptosendFeatherSharedSettings: DONE");
}   // END - preptosendFeatherSharedSettings()




