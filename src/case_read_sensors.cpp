/*
 * case_read_sensors.cpp
 * 
 */

#include "global.h"

void case_read_sensors()
{
    //debugPrintln("case_read_sensors() - executing");

    /*
     * ----------------------------------
     * Operate the SHT31 Temp & Humidity Sensor 
     * ----------------------------------
     */

    // TODO - are we supposed to turn on the SHT31's heater before taking a reading????

    //debugPrintln("case_read_sensors() - SHT31");

    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (!isnan(t))
    { // check if 'is not a number'
        myFeatherSettings.AIRTEMP = t;
        //debugPrint("case_read_sensors() - Air Temp (degC): ");
        //debugPrintlnFlt(t);
    }
    else
    {
        myFeatherSettings.AIRTEMP = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - ERROR Air Temp sensor read failed");
    }

    if (!isnan(h))
    { // check if 'is not a number'
        myFeatherSettings.HUMID = h;
        //debugPrint("case_read_sensors() - Air Humidity (%): ");
        //debugPrintlnFlt(h);
    }
    else
    {
        myFeatherSettings.HUMID = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - ERROR - Air Humidity sensor read failed");
    }

    /*
     * ----------------------------------
     * Operate the DS18B20 Temp Sensor 
     * ----------------------------------
     */
    //debugPrintln("case_read_sensors() - DS18B20");
    sensors.requestTemperatures();            // Send the command to get temperatures for all DS18B20's on the I2C bus.
    float tempC = sensors.getTempCByIndex(0); // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    if (tempC != DEVICE_DISCONNECTED_C)       // check if its a valid reading
    {
        myFeatherSettings.WATERTEMP = tempC;
        //debugPrint("case_read_sensors() - Water Temp (degC): ");
        //debugPrintlnFlt(tempC);
    }
    else
    {
        myFeatherSettings.WATERTEMP = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - ERROR - Water Temp sensor read failed");
    }


    /*
     * ----------------------------------
     * Operate the Ambient Light Sensor 
     * ----------------------------------
     */
    //debugPrintln("case_read_sensors() - AmbientLight");
    float lux = myLux.lightStrengthLux();
    if (lux != DEVICE_DISCONNECTED_C) // check if its a valid reading
    {
        myFeatherSettings.AMBIENTLIGHT = lux;
        //debugPrint("case_read_sensors() - Ambient Light (Lux): ");
        //debugPrintlnFlt(lux);
    }
    else
    {
        myFeatherSettings.AMBIENTLIGHT = -100; // set to an obvious failed magic number
        debugPrintln("case_read_sensors() - ERROR - Ambient Light sensor read failed");
    }

    assess_step = write_to_tft; // Set next state

    //debugPrintln("case_read_sensors() - done");
}