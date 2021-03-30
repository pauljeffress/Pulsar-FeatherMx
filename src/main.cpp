/*
 * main.cpp
 */

#include "global.h"        // My main header file for this project itself



// Create TFT instance
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  delay(3000); // wait for Serial to come up so we don't miss initial messages
  Serial.println("Pulsar FeatherMx"); 

  tft.begin();  // initialise the TFT screen
  tftdiags();   // read diagnostics (optional but can help debug problems)
  tft.setRotation(1);

  testText();
  delay(500);

}

void loop(void) {
  Serial.println("Hi");
  delay(500);
  
}