/*
 * case_write_to_tft.cpp
 * 
 */

#include "global.h"

void case_write_to_tft()
{
    debugPrintln("case_write_to_tft() - executing");
    
    // AIRTEMP
    tft.fillRect(215, 31, 90, 40, HX8357_BLACK); // 90 pixels Horiz, 40 pixels vert
    tft.setCursor(215, 70);
    tft.println(myfeatherSettings.AIRTEMP);

    // AIRHUMIDITY
    tft.fillRect(215, 71, 90, 40, HX8357_BLACK);
    tft.setCursor(215, 110);
    tft.println(myfeatherSettings.AIRHUMIDITY);

    // WATERTEMP
    tft.fillRect(215, 111, 90, 40, HX8357_BLACK);
    tft.setCursor(215, 150);
    tft.println(myfeatherSettings.WATERTEMP);

    // AMBIENTLIGHT
    tft.fillRect(215, 151, 130, 40, HX8357_BLACK);
    tft.setCursor(215, 190);
    tft.println(myfeatherSettings.AMBIENTLIGHT);


    assess_step = tickle_watchdog; // Set next state
}