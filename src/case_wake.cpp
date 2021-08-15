/*
 * case_wake.ino
 * 
 * Code to execute when doing wake
 * 
 * 
 */

 #include "global.h"
    
// ************************************************************************************************
// Wake from sleep

void case_wake()
{
      // YOU CANNOT WRITE TO SERIAL IN HERE, ITS CURRENTLY DISABLE/NOT-CONBFIGURED DUE TO PRE SLEEP ROUTINE
      
      

      
      // //Power up SRAM, turn on entire Flash
      // am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);
    
      // //Go back to using the main clock
      // //am_hal_stimer_int_enable(AM_HAL_STIMER_INT_OVERFLOW);
      // //NVIC_EnableIRQ(STIMER_IRQn);
      // am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
      // am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);
    
      // // Restore the TX/RX connections between the Artemis module and the CH340S on the Blackboard
      // am_hal_gpio_pinconfig(48 /* TXO-0 */, g_AM_BSP_GPIO_COM_UART_TX);
      // am_hal_gpio_pinconfig(49 /* RXI-0 */, g_AM_BSP_GPIO_COM_UART_RX);

      // // Reenable the debugger GPIOs
      // am_hal_gpio_pinconfig(20 /* SWDCLK */, g_AM_BSP_GPIO_SWDCK);
      // am_hal_gpio_pinconfig(21 /* SWDIO */, g_AM_BSP_GPIO_SWDIO);
  
      // //Turn on ADC
      // ap3_adc_setup();
      // ap3_set_pin_to_analog(busVoltagePin);

      // // Disable power for the gps (which could make the geofence interrupt pin bounce around causing false alerts)
      // gpsOFF();


      loop_step = loop_init;  // Always start with loop_init() when we come out of sleep.
    
}
