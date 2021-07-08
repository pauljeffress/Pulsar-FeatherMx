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

// Create SerialtoAGT - a new hw Serial port.
// Note I am calling it "Serial2", I'm not sure, but when I tried naming it something like "UartToAGT" I had problems tx/rx'ing.
// Define --- SERCOM    RX  TX      RX PAD           TX PAD
Uart Serial2 (&sercom0, 19, 18, SERCOM_RX_PAD_2, UART_TX_PAD_0);

// SerialTransfer initialisation
SerialTransfer STdriverF2A;  // create a SerialTransfer entity for the Feather to AGT connection.


/*============================*/
/* Global Variables           */
/*============================*/
featherSettings myfeatherSettings; // Create storage for the feather global settings in RAM

// iterationCounter is incremented each time a transmission is attempted.
// It helps keep track of whether messages are being sent successfully.
// It also indicates if the tracker has been reset (the count will go back to zero).
long iterationCounter = 0;

bool _printDebug = false; // Flag to show if message field debug printing is enabled. See debug_fns.ino
Stream *_debugSerial;     //The stream to send debug messages to (if enabled)

// state machine state trackers
volatile int loop_step = loop_init; // Holds state of the main loop() state machine
                                    // It's volatile as it needs to be maintained across the sleep/wake.
int assess_step = check_power;      // Holds state of the assess_situation state machine
uint32_t assess_iterations_counter = 0; // Useful while debugging my assess_situation state machine.
uint32_t assess_iterations_counter_last = 0; // Useful while debugging my assess_situation state machine.

bool send_F2Ablob = false;   
bool send_F2Pblob = false;   

bool sensor_sht31_status = BAD;         // Do we think the sensor is available/working or not?
bool sensor_ambientlight_status = BAD;
bool sensor_ds18b20_status = BAD;


bool feather_cant_tx_flag = false;  // control when the Feather can/can't TX to the AGT.
datum STDatumTX, STDatumRX;
uint32_t lastsend = 0; // used to determine when to send a dummy packet

/*============================*/
/* setup()
/*
/* Note: setup() code only runs when CPU has done a real powerup (or HW RESET), NOT a wake from sleep.
/*============================*/
void setup()
{
  // Note: Serial and many other things are not setup here.  Due to sleep/wake it is all done in init_loop()

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

  loop_step = loop_init; // Set openning state

} // END - setup()

/*============================*/
/* loop()
/*
/*============================*/
void loop(void)
{
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