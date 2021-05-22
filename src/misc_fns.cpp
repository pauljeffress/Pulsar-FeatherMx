/*
 * misc_fns.ino
 * 
 * Assorted functions
 * 
 * 
 */

#include "global.h"

void setupPins()
{
  debugPrintln("setupPins() - CODE STILL TO BE DONE IN HERE!!!!!!!!!");

  pinMode(LED_BUILTIN, OUTPUT); // Make the LED pin an output

  // pinMode(geofencePin, INPUT); // Configure the geofence pin as an input

  // pinMode(iridiumPwrEN, OUTPUT); // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  // digitalWrite(iridiumPwrEN, LOW); // Disable Iridium Power (HIGH = enable; LOW = disable)

  // pinMode(superCapChgEN, OUTPUT); // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  // digitalWrite(superCapChgEN, LOW); // Disable the super capacitor charger (HIGH = enable; LOW = disable)

  // pinMode(iridiumSleep, OUTPUT); // Iridium 9603N On/Off (Sleep) pin
  // digitalWrite(iridiumSleep, LOW); // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)

  // pinMode(iridiumRI, INPUT); // Configure the Iridium Ring Indicator as an input
  // pinMode(iridiumNA, INPUT); // Configure the Iridium Network Available as an input

  // pinMode(superCapPGOOD, INPUT); // Configure the super capacitor charger PGOOD input

  // pinMode(busVoltageMonEN, OUTPUT); // Make the Bus Voltage Monitor Enable an output
  // digitalWrite(busVoltageMonEN, LOW); // Set it low to disable the measurement to save power

  // control pins for serial connection to Feather
    pinMode(FEATHER_READY_TO_RX_PIN, OUTPUT);
    pinMode(AGT_WANTS_TO_TX_PIN, INPUT);
    digitalWrite(FEATHER_READY_TO_RX_PIN, LOW); // set initial state of this pin.
}

// help manage the control pins for serial connection to Feather
void setFEATHER_READY_TO_RX_PINhigh(){
  digitalWrite(FEATHER_READY_TO_RX_PIN, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}
void setFEATHER_READY_TO_RX_PINlow(){
  digitalWrite(FEATHER_READY_TO_RX_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}



// Beacuse Arduino print can't handle uint64_t I found this here https://forum.arduino.cc/t/printing-uint64_t/364646
// and it works well.
// Use it as follows;
//    Serial.print(my64toString(<your uint64_t variable>));
String my64toString(uint64_t x)  
{
     boolean flag = false; // For preventing string return like this 0000123, with a lot of zeros in front.
     String str = "";      // Start with an empty string.
     uint64_t y = 10000000000000000000;
     int res;
     if (x == 0)  // if x = 0 and this is not testet, then function return a empty string.
     {
           str = "0";
           return str;  // or return "0";
     }    
     while (y > 0)
     {                
            res = (int)(x / y);
            if (res > 0)  // Wait for res > 0, then start adding to string.
                flag = true;
            if (flag == true)
                str = str + String(res);
            x = x - (y * (uint64_t)res);  // Subtract res times * y from x
            y = y / 10;                   // Reducer y with 10    
     }
     return str;
}