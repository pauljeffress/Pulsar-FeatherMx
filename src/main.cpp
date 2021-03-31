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

  //testText();
  //delay(500);

  tft.fillScreen(HX8357_BLACK);
  tft.setTextColor(HX8357_GREEN);
  tft.setFont(&FreeSans18pt7b); // you need to #include each font before you can use it. https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf
  tft.setCursor(0, 26); // Needed to do this or the first row of my 18pt (approx 40 vert pixels per char) font was partly off top of TFT.

  tft.println("             Pulsar SubSystem ");
  tft.println("Temperature:");
  tft.println("Humidity:");
  tft.println("Iteration:");
  
  tft.setCursor(215, 70); 
  tft.println("23.5c");
  tft.setCursor(215, 110); 
  tft.println("86.2%");  
  tft.setCursor(215, 150); 
  tft.println("1");

  

}

void loop(void) {
  for (int i=0;i < 100;i++)
  {
    Serial.println(i);


  tft.print(i);

  delay(500);
  }
  
}