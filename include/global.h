/*
 * global.h
 * 
 * Overall header file for this project
 * 
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_GFX.h" 
#include "Adafruit_HX8357.h"

// defs for Adafruit 3.5" 480x320 TFT Featherwing - https://learn.adafruit.com/adafruit-3-5-tft-featherwing?view=all
// I have removed the pin defs for other boards.  See original example "graphicstest_featherwing.ino" for them.
// Anything else!
#if defined (__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined (__AVR_ATmega328P__) || \
defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
   #define STMPE_CS 6   // Touchscreen overlay controller CS pin - not in use in this project
   #define TFT_CS   9
   #define TFT_DC   10
   #define SD_CS    5   // SD slot CS pin - not in use in this project
#endif
#define TFT_RST -1

/* define any enums */


/* extern my global vars */
extern Adafruit_HX8357 tft;

/* function pre defines */
void tftdiags();
unsigned long testText();

#endif