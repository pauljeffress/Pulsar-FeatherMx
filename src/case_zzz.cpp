/*
 * case_zzz.ino
 * 
 * Code to execute when doing zzz
 * 
 * 
 */

 #include "global.h"

// ************************************************************************************************
// Go to sleep

void case_zzz()
{
      debugPrintln("case_zzz() executing");
      
      debugPrintln("case_zzz() - Getting ready to put the FeatherMx into deep sleep...");

      debugPrintln("case_zzz() - WARNING CODE NOT FINISHED IN HERE");


      // TODO
      //  - disable power to lots of things
      //  - close serial ports e.g. iridiumSerial.end();
      //  - Close the I2C port e.g.Wire1.end();
      //digitalWrite(busVoltageMonEN, LOW); // Disable the bus voltage monitor
      //digitalWrite(LED, LOW); // Disable the LED

      disableDebugging(); // Disable the serial debug messages

      disableLogging(); // Disable the serial logging messages going to the OLA    

      // Close and detach the serial console
      Serial.println(F("Going to SLEEP until next WAKEINT..."));
      Serial.flush(); //Finish any prints
      Serial.end(); // Close the serial console

      // // Code taken (mostly) from the LowPower_WithWake example and the and OpenLog_Artemis PowerDownRTC example
      
      // // Turn off ADC
      // power_adc_disable();
        
      // // Disabling the debugger GPIOs saves about 1.2 uA total:
      // am_hal_gpio_pinconfig(20 /* SWDCLK */, g_AM_HAL_GPIO_DISABLE);
      // am_hal_gpio_pinconfig(21 /* SWDIO */, g_AM_HAL_GPIO_DISABLE);
  
      // // These two GPIOs are critical: the TX/RX connections between the Artemis module and the CH340S
      // // are prone to backfeeding each other. To stop this from happening, we must reconfigure those pins as GPIOs
      // // and then disable them completely:
      // am_hal_gpio_pinconfig(48 /* TXO-0 */, g_AM_HAL_GPIO_DISABLE);
      // am_hal_gpio_pinconfig(49 /* RXI-0 */, g_AM_HAL_GPIO_DISABLE);
  
      // //Power down Flash, SRAM, cache
      // am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);         //Turn off CACHE
      // am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_512K);    //Turn off everything but lower 512k
      // //am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_64K_DTCM); //Turn off everything but lower 64k? Be careful here. "Global variables use 56180 bytes of dynamic memory."

      // //Keep the 32kHz clock running for RTC
      // am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
      // am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

      // geofence_alarm = false; // The geofence alarm pin will have been bouncing around so let's make sure the flag is clear

      // // ASLEEP in this while loop
      // // -------------------------
      // // This while loop keeps the processor asleep until WAKEINT seconds have passed
      // // Or a geofence alarm takes place (but only if geofence alarms are enabled)
      // while ((interval_alarm == false) && ((geofence_alarm == false) || ((myTrackerSettings.FLAGS2 & FLAGS2_GEOFENCE) != FLAGS2_GEOFENCE)))
      // {
      //   // Go to Deep Sleep.
      //   am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); 
      // }

      // WAKE UP and do stuff
      // --------------------
      //interval_alarm = false; // Clear the interval alarm flag now 
      

      // Wake up!
      loop_step = wake;

}
