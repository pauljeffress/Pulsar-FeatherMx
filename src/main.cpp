/*
 * main.cpp
 */

#include "global.h"        // My main header file for this project itself

// Create TFT instance
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

// Create SHT31 instance
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // using default i2c addr of 0x44

// Create Ambient Light Sensor instance
// EN = 13, SDA = 5, SCL = 6 - I am not using EN, its actually hard wired high.
DFRobot_B_LUX_V30B myLux(13,6,5);  // using default i2c addr of 0x4A

// Create DS18B20 temp sensor
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);



void setup() {
  Serial.begin(115200);
  delay(5000); // wait for Serial to come up so we don't miss initial messages
  Serial.println("Pulsar FeatherMx"); 
  tftSetup();
  sensorsSetup();
  Serial.println("Setup() complete");
}

void loop(void) {

  sensorsTest();

  delay(1000);  
}