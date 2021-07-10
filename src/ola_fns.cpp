/*
 * ola_fns.cpp
 *
 * Supporting functions for logging to the OpenLog Artemis (OLA).
 * 
 */

#include "global.h"        // My main header file for this project itself

void olaHeartbeat()
{
    Serial.println("doing olaHeartbeat()");

    // send the basic heartbeat message to the OLA.
    Serial3.print("FeatherMx Heartbeat - FeatherMX millis() = ");
    Serial3.println(millis());
}