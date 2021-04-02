/*
 * tft_support.cpp
 */

#include "global.h"        // My main header file for this project itself

void sensorsSetup()
{
  // Setup the SHT31 sensor
  Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");
  Serial.println("SHT31 Setup done");

  // setup ambient light sensor dome
  myLux.begin();  // currently commented out as it is stopping the SHT31 from working.
  Serial.println("Ambient Setup done");

  // Start up the DallasTemperature library
  sensors.begin();
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  Serial.println("DS18B20 Setup done");

}

void sensorsTest()
{
  // My early code to test all sensors could be read and displayed on the TFT.
  
  /*
  * ----------------------------------
  * Operate the SHT31 Temp & Humidity Sensor 
  * ----------------------------------
  */
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("SHT Temp *C = "); Serial.print(t); Serial.print("\t");
    tft.fillRect(215,31,90,40,HX8357_BLACK);  // 90 pixels Horiz, 40 pixels vert
    tft.setCursor(215, 70); 
    tft.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("SHT Hum. % = "); Serial.print(h); Serial.print("\t"); 
    tft.fillRect(215,71,90,40,HX8357_BLACK);
    tft.setCursor(215, 110); 
    tft.println(h);  
  } else { 
    Serial.println("Failed to read humidity");
  }

/*
 * ----------------------------------
 * Operate the DS18B20 Temp Sensor 
 * ----------------------------------
 */
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);
  // Check if reading was successful
  if(tempC != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("DS18B20 Temp *C = "); Serial.print(tempC); Serial.print("\t"); 
    tft.fillRect(215,111,90,40,HX8357_BLACK);
    tft.setCursor(215, 150); 
    tft.println(tempC); 
  } 
  else
  {
    Serial.println("Error: Could not read temperature data");
  }

/*
 * ----------------------------------
 * Operate the Ambient Light Sensor 
 * ----------------------------------
 */
  float lux = myLux.lightStrengthLux();
  Serial.print("Light = ");Serial.print(lux);Serial.println(" (lux).");
  tft.fillRect(215,151,130,40,HX8357_BLACK);
  tft.setCursor(215, 190); 
  tft.println(lux); 


}