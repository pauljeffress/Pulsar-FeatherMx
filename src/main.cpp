/*
 * main.cpp
 */

/*
 * This is my "Pulsar FeatherMx" PlatformIO project for the Feather M4 in the boat.
 *
 */

#include "global.h" // My main header file for this project itself

/*============================*/
/* Global object declarations */
/*============================*/
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST); // Create TFT instance
Adafruit_SHT31 sht31 = Adafruit_SHT31();                        // Create SHT31 instance using default i2c addr of 0x44
// Create Ambient Light Sensor instance
// EN = 13, SDA = 5, SCL = 6 - I am not using EN, its actually hard wired high.
DFRobot_B_LUX_V30B myLux(13, 6, 5); // using default i2c addr of 0x4A
// Create DS18B20 temp sensor
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Create Serial2 - a new hw Serial port I'll use to connect to the AGT.
// Note I am calling it "Serial2", I'm not sure, but when I tried naming it something like "UartToAGT" I had problems tx/rx'ing.
// Define --- SERCOM    RX  TX      RX PAD           TX PAD
Uart Serial2 (&sercom0, 19, 18, SERCOM_RX_PAD_2, UART_TX_PAD_0);

// Create Serial3 - a new hw Serial port I'll use to connect to the OpenLog Artemis.
// Define --- SERCOM    RX  TX      RX PAD           TX PAD
Uart Serial3 (&sercom3, 11, 12, SERCOM_RX_PAD_3, UART_TX_PAD_0);

// SerialTransfer initialisation
SerialTransfer STdriverF2A;  // create a SerialTransfer entity for the Feather to AGT connection.


// Depending on the board, you can select SAMD21 Hardware Timer from TC3-TCC
// SAMD21 Hardware Timer from TC3 or TCC
// SAMD51 Hardware Timer only TC3
// xxxx - SAMDTimer ITimer0(TIMER_TC3);   




/*============================*/
/* Global Variables           */
/*============================*/
FeatherSharedSettings myFeatherSharedSettings;  // My RAM copy of the settings I will share with the AGT
AgtSharedSettings myAgtSharedSettings;   // My RAM copy of the setting the AGT has shared with me.


// state machine state trackers
volatile int loop_step = loop_init; // Holds state of the main loop() state machine
                                    // It's volatile as it needs to be maintained across the sleep/wake.
int assess_step = check_power;      // Holds state of the assess_situation state machine

// Action Flags
bool flag_do_agt_tx = false;   // Set when something wants the Feather to send it's sharedSettings to the AGT

// Sensor Status Flags
bool sensor_sht31_status = BAD;         // Do we think the sensor is available/working or not?
bool sensor_ambientlight_status = BAD;
bool sensor_ds18b20_status = BAD;

// stuff for my printDebug functionality (most code is in debug_fns.cpp)
bool _printLog = false; // Flag to show if message field log printing is enabled. 
Stream *_logSerial;     //The stream to send log messages to (if enabled)

// xxx - not needed anymore - bool feather_cant_tx_flag = false;  // control when the Feather can/can't TX to the AGT.
//datum STDatumTX, STDatumRX;
//uint32_t lastsend = 0; // used to determine when to send a dummy packet


/*============================
 * setup()
 *
 * Note: setup() code only runs when CPU has done a real powerup (or HW RESET), NOT a wake from sleep.
 ============================*/
void setup()
{
  // Note: Serial and many other things are not setup here.  Due to sleep/wake it is all done in init_loop()

  myCANid = CBP_CANID_FEATHERMX; // Set myCANid based on defines in CBP.h

  // xxx - should this be done in loop_init???
  setupPins(); // initialise all GPIOs

  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);  


  disableDebugging(); // Make sure the serial debug messages are disabled until the Serial port is open ( see loop_init() )!
  disableLogging(); // Make sure the serial logging messages (to OLA) are disabled until the Serial port is open ( see loop_init() )!

  // note, below will run but Serial Ports are not open yet so no debug output will be shown.
  initFeatherMxSettings();        // Initialise the myFeatherMxSettings, from defaults or otherwise.
  initFeatherSharedSettings();  // Initialise the myFeatherSharedSettings, from myFeatherMxSettings.FMX_
  initTFTFeatherInternalSettings();
  initPowerFeatherSettings();

  loop_step = loop_init; // Set openning state

} // END - setup()

/*============================
 * loop()
 *
 *============================*/
void loop(void)
{
  // Do LED
  digitalWrite( LED_BUILTIN, millis() % 500 > 250 );

  // loop is one big state machine that controls the sequencing of the code
  switch (loop_step)
  {
  // ************************************************************************************************
  // Initialise things
  case loop_init:
    case_loop_init();
    break;

  // ************************************************************************************************
  // Assess situation
  case assess_situation:
    case_assess_situation();
    break;

  // ************************************************************************************************
  // Go to sleep
  case zzz:
    case_zzz();
    break;

  // ************************************************************************************************
  // Wake from sleep
  case wake:
    case_wake();
    break;

  // ************************************************************************************************
  // DEFAULT - should not happen, but programing it defensively
  default:
    debugPrintln("ERROR - we hit the default: in main state machine");
    loop_step = loop_init; // go to loop_init to try and clean up from whatever caused this!
    break;

  } // End of switch (loop_step)
} // END - loop()