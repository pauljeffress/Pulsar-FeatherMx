/*
 * main.cpp
 */

#include "global.h"        // My main header file for this project itself



// Create TFT instance
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

// Create SHT31 instance
//DFRobot_SHT3x   sht3x;
//DFRobot_SHT3x sht3x(&Wire,/*address=*/0x45,/*RST=*/4);
Adafruit_SHT31 sht31 = Adafruit_SHT31();

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

  // Initialise the SHT31
    // while (sht3x.begin() != 0) {
    //   Serial.println("Failed to Initialize the chip, please confirm the wire connection");
    //   delay(1000);
    // }
    // Serial.print("Chip serial number");
    // Serial.println(sht3x.readSerialNumber());

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


}

void loop(void) {

  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
    //tft.setCursor(215, 70);
    tft.fillRect(215,30,200,40,HX8357_BLACK);
    tft.setCursor(215, 70); 
    tft.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
    //tft.setCursor(215, 110); 
    tft.fillRect(215,70,200,40,HX8357_BLACK);
    tft.setCursor(215, 110); 
    tft.println(h);  
  } else { 
    Serial.println("Failed to read humidity");
  }

  delay(1000);

  // for (int i=0;i < 100;i++)
  // {
  //   Serial.println(i);
  //   tft.print(i);
  //   delay(500);
  // }
  
}