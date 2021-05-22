/*
 * serial_fns.cpp
 */

#include "global.h" // My main header file for this project itself

// Part of the new hw Serial port I create on the Feather to talk to the AGT.
// The adafruit document said for SAMD51 I needed to define all 5 interupt handlers for SERCOM0
void SERCOM0_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM0_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM0_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM0_3_Handler()
{
  Serial2.IrqHandler();
}


void serialSetup()
{
    // Initialise the Serial that connects this Feather to the AGT
    Serial2.begin(9600); // AGT end is using SoftwareSerial, so go slower speed.
    // Reassign pins on the internal SAMD pinmux, to connect to my SERCOMs. They may have defaulted to other peripherals.
    // Assign pins 10 & 11 SERCOM functionality
    pinPeripheral(18, PIO_SERCOM_ALT); // the 'PIO_SERCOM' should be 'PIO_SERCOM_ALT' if we are trying to use the 'alternate' pins for this.
    pinPeripheral(19, PIO_SERCOM_ALT); // same as above comment.

    // Initialise FNIC SerialTransfer drivers
    STdriverF2A.begin(Serial2, true, Serial);

    // prep datum for first use
    STDatumTX.i1 = 1;
    STDatumTX.c1 = 'F';
    STDatumTX.c2 = 't';
    STDatumTX.c3 = 'o';
    STDatumTX.c4 = 'A';

    feather_cant_tx_flag = false;
}