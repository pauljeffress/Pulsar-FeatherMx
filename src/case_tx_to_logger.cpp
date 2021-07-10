/*
 * case_tx_to_logger.cpp
 * 
 */

#include "global.h"

void case_tx_to_logger()
{
    debugPrintln("case_tx_to_logger() - executing");

    olaHeartbeat(); // Send a heartbeat to the OLA

    assess_step = tickle_watchdog; // Set next state
}