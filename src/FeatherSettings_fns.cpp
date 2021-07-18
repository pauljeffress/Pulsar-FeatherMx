/*
 * FeatherSettings_fns.cpp
 */

#include "global.h" // My main header file for this project itself

void printFeatherSettings(void)
// Print the tracker settings
{
    Serial.println("FeatherSettings are:");
    Serial.print("SWVER: ");
    Serial.print((myFeatherSettings.SWVER & 0xf0) >> 4);
    Serial.print(".");
    Serial.println(myFeatherSettings.SWVER & 0x0f);
    Serial.print("BATTV: ");
    Serial.println(((float)myFeatherSettings.BATTV / 100),2);
    Serial.print("PRESS: ");
    Serial.println(myFeatherSettings.PRESS);
    Serial.print("AIRTEMP: ");
    Serial.println(((float)myFeatherSettings.AIRTEMP / 100),2);
    Serial.print("HUMID: ");
    Serial.println(((float)myFeatherSettings.HUMID / 100),2);
    Serial.print("WATERTEMP: ");
    Serial.println(((float)myFeatherSettings.WATERTEMP / 100),2);
    Serial.print("AMBIENTLIGHT: ");
    Serial.println(myFeatherSettings.AMBIENTLIGHT);
    
    Serial.print("YEAR: ");
    Serial.println(myFeatherSettings.GPSYEAR);
    Serial.print("MONTH: ");
    Serial.println(myFeatherSettings.GPSMONTH);
    Serial.print("DAY: ");
    Serial.println(myFeatherSettings.GPSDAY);
    Serial.print("HOUR: ");
    Serial.println(myFeatherSettings.GPSHOUR);
    Serial.print("MIN: ");
    Serial.println(myFeatherSettings.GPSMIN);
    Serial.print("SEC: ");
    Serial.println(myFeatherSettings.GPSSEC);
    Serial.print("MILLIS: ");
    Serial.println(myFeatherSettings.GPSMILLIS);

    Serial.print("CUSTOMMODE: ");
    Serial.println(myFeatherSettings.CUSTOMMODE);
    Serial.print("SYSTEMSTATUS: ");
    Serial.println(myFeatherSettings.SYSTEMSTATUS);   

    Serial.print("GPSTIMESTAMP: ");
    Serial.println("XXX - doesnt print properly");      // myFeatherSettings.GPSTIMESTAMP   
    Serial.print("LAT: ");
    Serial.println(((float)myFeatherSettings.LAT / 10000000),7);
    Serial.print("LON: ");
    Serial.println(((float)myFeatherSettings.LON / 10000000),7);
    Serial.print("ALT: ");
    Serial.println(((float)myFeatherSettings.ALT / 1000),3);
    Serial.print("SPEED: ");
    Serial.println(((float)myFeatherSettings.SPEED / 1000),3);
    Serial.print("HEAD: ");
    Serial.println(((float)myFeatherSettings.HEAD / 10000000),1);
    Serial.print("SATS: ");
    Serial.println(myFeatherSettings.SATS);
    Serial.print("PDOP: ");
    Serial.println(((float)myFeatherSettings.PDOP / 100),2);
    Serial.print("FIX: ");
    Serial.println(myFeatherSettings.FIX);
    
    Serial.print("WAKEINT: ");
    Serial.println(myFeatherSettings.WAKEINT);
    Serial.print("TXAGTINT: ");
    Serial.println(myFeatherSettings.TXAGTINT);
    Serial.print("RXAPINT: ");
    Serial.println(myFeatherSettings.RXAPINT);
    Serial.print("LOWBATT: ");
    Serial.println(((float)myFeatherSettings.LOWBATT / 100),2);
} // END - printFeatherSettings()


void initFeatherSettings(void) // Initialises the myFeatherSettings in RAM with the default values
{
  myFeatherSettings.STX = DEF_STX;
  myFeatherSettings.SWVER = DEF_SWVER;
  myFeatherSettings.BATTV = DEF_BATTV;
  myFeatherSettings.PRESS = DEF_PRESS;
  myFeatherSettings.AIRTEMP = DEF_AIRTEMP;
  myFeatherSettings.HUMID = DEF_HUMID;
  myFeatherSettings.WATERTEMP = DEF_WATERTEMP;  
  myFeatherSettings.AMBIENTLIGHT = DEF_AMBIENTLIGHT;

  myFeatherSettings.GPSYEAR = DEF_GPSYEAR;
  myFeatherSettings.GPSMONTH = DEF_GPSMONTH;
  myFeatherSettings.GPSDAY = DEF_GPSDAY;
  myFeatherSettings.GPSHOUR = DEF_GPSHOUR;
  myFeatherSettings.GPSMIN = DEF_GPSMIN;
  myFeatherSettings.GPSSEC = DEF_GPSSEC;
  myFeatherSettings.GPSMILLIS = DEF_GPSMILLIS;

  myFeatherSettings.CUSTOMMODE = DEF_CUSTOMMODE;
  myFeatherSettings.SYSTEMSTATUS = DEF_SYSTEMSTATUS;

  myFeatherSettings.GPSTIMESTAMP = DEF_GPSTIMESTAMP;
  myFeatherSettings.LAT = DEF_LAT;
  myFeatherSettings.LON = DEF_LON;
  myFeatherSettings.ALT = DEF_ALT;
  myFeatherSettings.SPEED = DEF_SPEED;
  myFeatherSettings.HEAD = DEF_HEAD;
  myFeatherSettings.SATS = DEF_SATS;
  myFeatherSettings.PDOP = DEF_PDOP;
  myFeatherSettings.FIX = DEF_FIX;

  myFeatherSettings.WAKEINT = DEF_WAKEINT;
  myFeatherSettings.TXAGTINT = DEF_TXAGTINT;
  myFeatherSettings.RXAPINT = DEF_RXAPINT;
  myFeatherSettings.LOWBATT = DEF_LOWBATT;
  myFeatherSettings.ETX = DEF_ETX;
  debugPrintln("initFeatherSettings: RAM tracker settings initialised");
}   // END - initFeatherSettings()