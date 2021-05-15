/*
 * case_read_sensors.cpp
 * 
 */

#include "global.h"

void case_read_sensors()
{
    debugPrintln("case_read_sensors() - executing");

    /*
     * ----------------------------------
     * Operate the SHT31 Temp & Humidity Sensor 
     * ----------------------------------
     */

    // TODO - are we supposed to turn on the SHT31's heater before taking a reading????

    debugPrintln("case_read_sensors() - SHT31");

    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (!isnan(t))
    { // check if 'is not a number'
        myfeatherSettings.AIRTEMP = t;
        debugPrintln("case_read_sensors() - Air Temp (degC): ");
        debugPrintlnFlt(t);

        //tft.fillRect(215, 31, 90, 40, HX8357_BLACK); // 90 pixels Horiz, 40 pixels vert
        //tft.setCursor(215, 70);
        //tft.println(t);
    }
    else
    {
        myfeatherSettings.AIRTEMP = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - Air Temp (degC): ERROR");
    }

    if (!isnan(h))
    { // check if 'is not a number'
        myfeatherSettings.AIRHUMIDITY = h;
        debugPrintln("case_read_sensors() - Air Humidity (%): ");
        debugPrintlnFlt(h);

        //tft.fillRect(215, 71, 90, 40, HX8357_BLACK);
        //tft.setCursor(215, 110);
        //tft.println(h);
    }
    else
    {
        myfeatherSettings.AIRHUMIDITY = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - Air Humidity (%): ERROR");
    }

    /*
     * ----------------------------------
     * Operate the DS18B20 Temp Sensor 
     * ----------------------------------
     */
    debugPrintln("case_read_sensors() - DS18B20");
    sensors.requestTemperatures();            // Send the command to get temperatures for all DS18B20's on the I2C bus.
    float tempC = sensors.getTempCByIndex(0); // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    if (tempC != DEVICE_DISCONNECTED_C)       // check if its a valid reading
    {
        myfeatherSettings.WATERTEMP = tempC;
        debugPrintln("case_read_sensors() - Water Temp (degC): ");
        debugPrintlnFlt(tempC);

        //tft.fillRect(215, 71, 90, 40, HX8357_BLACK);
        //tft.setCursor(215, 110);
        //tft.println(h);
    }
    else
    {
        myfeatherSettings.WATERTEMP = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - Water Temp (degC): ERROR");
    }

    //tft.fillRect(215, 111, 90, 40, HX8357_BLACK);
    //tft.setCursor(215, 150);
    //tft.println(tempC);

    /*
     * ----------------------------------
     * Operate the Ambient Light Sensor 
     * ----------------------------------
     */
    debugPrintln("case_read_sensors() - AmbientLight");
    float lux = myLux.lightStrengthLux();
    if (lux != DEVICE_DISCONNECTED_C) // check if its a valid reading
    {
        myfeatherSettings.AMBIENTLIGHT = lux;
        debugPrintln("case_read_sensors() - Ambient Light (Lux): ");
        debugPrintlnFlt(lux);

        //tft.fillRect(215, 71, 90, 40, HX8357_BLACK);
        //tft.setCursor(215, 110);
        //tft.println(h);
    }
    else
    {
        myfeatherSettings.AMBIENTLIGHT = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - Ambient Light (Lux): ERROR");
    }

    //tft.fillRect(215, 151, 130, 40, HX8357_BLACK);
    //tft.setCursor(215, 190);
    //tft.println(lux);

    assess_step = write_to_tft; // Set next state
}