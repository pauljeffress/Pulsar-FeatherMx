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

    // // Check if its ACKnowledged by the AGT sending back its Datum.
    // debugPrintln("sendSharedSettings_to_AGT() - waiting for Datum from AGT");
    // uint32_t time_limit_millis = 5000;  // 5 seconds - only wait this long
    // uint32_t start_time_millis = millis();
    // while ((!success)  && (millis() < (start_time_millis + time_limit_millis)))     // until we succed or timeout
    // {
    //     if (STdriverF2A.available())    // is there a Datum from the AGT?
    //     {
    //         STdriverF2A.rxObj(myAgtSharedSettings); // get the Datum
    //         debugPrintln("sendSharedSettings_to_AGT() - received response Datum from AGT");
    //         seconds_since_last_agt_rx = 0; // reset this counter as we have just received from the AGT.
    //         success = true;
    //     }
    // }

    // if (success)    // We did get a Datum, so we need to take action
    // {
    //     // xxx - add code to either flag or just start processing the received
    // }
    // else
    // {
    //     debugPrintln("sendSharedSettings_to_AGT() - ERROR - did not receive response Datum from AGT!");
    // }
    return(success);
}   // END - sendSharedSettings_to_AGT


void initFeatherSharedSettings(void) // Initialises the myFeatherSharedSettings in RAM with the default values
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
  myFeatherSharedSettings.SEC = myFeatherSettings.SEC;
  myFeatherSharedSettings.MILLIS = myFeatherSettings.MILLIS;

  myFeatherSharedSettings.GPSTIMESTAMP = myFeatherSettings.GPSTIMESTAMP;
  
  // myFeatherSharedSettings.LAT_PA_16 = int32_to_int16_a(myFeatherSettings.LAT);
  // myFeatherSharedSettings.LAT_PB_16 = int32_to_int16_b(myFeatherSettings.LAT);
  // myFeatherSharedSettings.LAT_PC_16 = int32_to_int16_c(myFeatherSettings.LAT);
  // debugPrint("LAT_PA_16="); Serial.println(myFeatherSharedSettings.LAT_PA_16);
  // debugPrint("LAT_PB_16="); Serial.println(myFeatherSharedSettings.LAT_PB_16);
  // debugPrint("LAT_PC_16="); Serial.println(myFeatherSharedSettings.LAT_PC_16);
  // int32_t temp = myFeatherSharedSettings.LAT_PA_16*((((uint32_t)myFeatherSharedSettings.LAT_PB_16) << 16) | ((uint32_t)myFeatherSharedSettings.LAT_PC_16));
  // debugPrint("LAT="); Serial.println(temp);
  
  myFeatherSharedSettings.LAT = myFeatherSettings.LAT;
  myFeatherSharedSettings.LON = myFeatherSettings.LON;
  myFeatherSharedSettings.ALT = myFeatherSettings.ALT;
  myFeatherSharedSettings.SPEED = myFeatherSettings.SPEED;
  myFeatherSharedSettings.HEAD = myFeatherSettings.HEAD;
  myFeatherSharedSettings.SATS = myFeatherSettings.SATS;
  myFeatherSharedSettings.PDOP = myFeatherSettings.PDOP;
  myFeatherSharedSettings.FIX = myFeatherSettings.FIX;

  debugPrintln("initFeatherSharedSettings: RAM settings initialised");
}   // END - initFeatherSharedSettings()


void preptosendFeatherSharedSettings(void) // gets myFeatherSharedSettings fresh (mainly from myFeatherSettings) and ready to TX to AGT.
{
  initFeatherSharedSettings(); // start by setting everything to align with the current myFeatherSettings
  
  // then alter anything that needs to change just before transmission to the AGT.
  // xxx TBD

  debugPrintln("preptosendFeatherSharedSettings: DONE");
}   // END - preptosendFeatherSharedSettings()



int16_t int32_to_int16_a(int32_t input_int32)
{
  if (input_int32 < 0) return(-1);
  else return(1);
}

uint16_t int32_to_int16_b(int32_t input_int32)
{
  return((uint16_t) (((uint32_t)abs(input_int32)) >> 16));
}

uint16_t int32_to_int16_c(int32_t input_int32)
{
  // int32_t x = abs(input_int32);
  // Serial.println(x);
  // int32_t y = (uint32_t)abs(input_int32);
  // Serial.println(y);
  // uint32_t z = ((uint32_t)abs(input_int32)) & 0x0000FFFFuL;
  // Serial.println(z);
  // uint16_t zz = (uint16_t) (((uint32_t)abs(input_int32)) & 0x0000FFFFuL);
  // Serial.println(zz);  
  return((uint16_t) (((uint32_t)abs(input_int32)) & 0x0000FFFFuL));
}

