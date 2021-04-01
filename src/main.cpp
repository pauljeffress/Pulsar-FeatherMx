/*
 * main.cpp
 */

#include "global.h"        // My main header file for this project itself



// Create TFT instance
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

// Create SHT31 instance
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Create Ambient Light Sensor instance
DFRobot_B_LUX_V30B    myLux(13,6,5);

// Create DS18B20 temp sensor
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);



void setup() {
  Serial.begin(115200);
  delay(5000); // wait for Serial to come up so we don't miss initial messages
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


  // setup ambient light sensor dome
  //myLux.begin();  // currently commented out as it is stopping the SHT31 from working.
 

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


  Serial.println("Setup() complete");
}

void loop(void) {

/*
 * ----------------------------------
 * Operate the SHT31 Temp & Humidity Sensor 
 * ----------------------------------
 */
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("SHT Temp *C = "); Serial.print(t); Serial.print("\t\t");
    //tft.setCursor(215, 70);
    tft.fillRect(215,30,200,40,HX8357_BLACK);
    tft.setCursor(215, 70); 
    tft.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("SHT Hum. % = "); Serial.print(h); Serial.print("\t\t");
    //tft.setCursor(215, 110); 
    tft.fillRect(215,70,200,40,HX8357_BLACK);
    tft.setCursor(215, 110); 
    tft.println(h);  
  } else { 
    Serial.println("Failed to read humidity");
  }

/*
 * ----------------------------------
 * Operate the Ambient Light Sensor 
 * ----------------------------------
 */
  //Serial.print("value: ");
  //Serial.print(myLux.lightStrengthLux());
  //Serial.println(" (lux).");

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
    Serial.print("DS18B20 Temp *C = ");
    Serial.println(tempC);
  } 
  else
  {
    Serial.println("Error: Could not read temperature data");
  }


  delay(1000);  
}