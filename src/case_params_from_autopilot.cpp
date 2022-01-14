/*
 * case_params_from_autopilot.cpp
 * 
 */

#include "global.h"

void case_params_from_autopilot()
{
    // if its time to request params via MAVLink from the AutoPilot
    if (seconds_since_last_mavlink_req > MAVLINKREQUESTPERIODSECONDS)
    {
        #ifdef MAVLINK_DEBUG 
            debugPrintln("case_params_from_autopilot() - Time to execute.");
        #endif

        // Prep source and dest MAVLink addressing info, to be used in below actions.
        uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
        uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
        uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
        uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

        
        // Try to minimise the noise on the MAVLink port
        // =============================================
        // Note this is a bit out of place, but in a bit of a stateless way I'm just trying to, 
        // ensure that when the main code in this function is executed, I have recently squashed as much noise as possible.  The noise can be
        // due to Mission Planner telling the AP to stream various messages, or other left over crud from myself.  The point is though to quieten
        // as much of it down now as possible before moving on. This is a bit of a hack, but it does help me, particularly when troubleshooting.
        // I also try to minimise what the AutoPilot streams with 

        mavlink_unrequest_datastream(); // try to silence a common source (setoff by Mission Planner!!!!)


        // Request just the messages I am interested in at suitable interval
        // =================================================================

        // I tried to use the MAV_CMD_REQUEST_MESSAGE method in a COMMAND_LONG (#76) (https://mavlink.io/en/messages/common.html#COMMAND_LONG) but I never
        // seemed to get a response, or even a COMMAND_ACK (#77) https://mavlink.io/en/messages/common.html#COMMAND_ACK back.
        // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
        //uint16_t _cl_command      = MAV_CMD_REQUEST_MESSAGE; // MAV_CMD_SET_MESSAGE_INTERVAL; // https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
        //uint8_t _cl_confirmation  = 0; // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
        //float   _cl_param1 = MAVLINK_MSG_ID_POWER_STATUS;   // MAVLink Message ID
        //float   _cl_param2 = 0; // Not used.
        //float   _cl_param3 = 0; // Not used, so set to zero.
        //float   _cl_param4 = 0; // Not used, so set to zero.
        //float   _cl_param5 = 0; // Not used, so set to zero.
        //float   _cl_param6 = 0; // Not used, so set to zero.
        //float   _cl_param7 = 0; // Not used, so set to zero.
        
        // So insted I switched to using the MAV_CMD_SET_MESSAGE_INTERVAL method in a COMMAND_LONG (#76) (https://mavlink.io/en/messages/common.html#COMMAND_LONG) 
        // as it seemed to work.  The nominated messages would start flowing and I would also get the correct 
        // COMMAND_ACK (#77) https://mavlink.io/en/messages/common.html#COMMAND_ACK back from the AutoPilot, acknowledging the command.
        
        // Build 1st COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL message.
        // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
        uint16_t _cl_command      = MAV_CMD_SET_MESSAGE_INTERVAL; // https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
        uint8_t _cl_confirmation  = 0; // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
        float   _cl_param1 = MAVLINK_MSG_ID_POWER_STATUS;   // MAVLink Message ID
        float   _cl_param2 = 1000000; // Interval (uS) between messages e.g. 1sec interval = 1000000uS
        float   _cl_param3 = 0; // Not used, so set to zero.
        float   _cl_param4 = 0; // Not used, so set to zero.
        float   _cl_param5 = 0; // Not used, so set to zero.
        float   _cl_param6 = 0; // Not used, so set to zero.
        float   _cl_param7 = 0; // Not used, so set to zero.

        // Initialize the required buffers
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        // Pack and send the 1st message
        // Request MAVLINK_MSG_ID_POWER_STATUS (#125) - https://mavlink.io/en/messages/common.html#POWER_STATUS
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500); // give AP 1/2 sec to process before we send next CMD, don't want to DOS it.

        // Modify the COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL message for the next message I want to request, then pack and send it.
        // Request MAVLINK_MSG_ID_HWSTATUS (#165) - https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS
        _cl_param1 = MAVLINK_MSG_ID_HWSTATUS;   // MAVLink Message ID
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500);

        // Request MAVLINK_MSG_ID_GPS_RAW_INT (#24) - https://mavlink.io/en/messages/common.html#GPS_RAW_INT
        _cl_param1 = MAVLINK_MSG_ID_GPS_RAW_INT;   // MAVLink Message ID
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500);

        // Request MAVLINK_MSG_ID_GLOBAL_POSITION_INT (#33) - https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
        _cl_param1 = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;   // MAVLink Message ID
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500);

        // Request MAVLINK_MSG_ID_SYS_STATUS (#1) - https://mavlink.io/en/messages/common.html#SYS_STATUS
        _cl_param1 = MAVLINK_MSG_ID_SYS_STATUS;   // MAVLink Message ID
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500);

        // Request MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT (#62) - https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
        _cl_param1 = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;   // MAVLink Message ID
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500);

        // Request MAVLINK_MSG_ID_BATTERY_STATUS (#147) - https://mavlink.io/en/messages/common.html#BATTERY_STATUS
        _cl_param1 = MAVLINK_MSG_ID_BATTERY_STATUS;   // MAVLink Message ID
        mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len); //Write data to serial port byte by byte.
        delay(500);


        // Collect the messages I am interested in as per requests above
        // =============================================================

        // Initialise temp holding spots and put a magic number in them so I can know when they have been updated successfully

        // Do a bulk collection of data from messages
        // then here I need to check for the cmd response (which should be a COMMAND_ACK 
        // according to https://mavlink.io/en/services/command.html) and for a message with the message# I want.
        debugPrint("case_params_from_autopilot() - ATTEMPTING RX - starting at Millis:"); Serial.println(millis());
        uint32_t start = millis();  // xxx - need to review how I'm timing this loop...seems clunky. Also need to take any contants and set the as #defines.
        while ((millis() < (start + 3000)) && (millis() > 4000))  // For 3 seconds, see if we can assemble received msgs and if so process them.
            mavlink_receive(); 
        seconds_since_last_ap_rx = 0;    // reset counter
        debugPrint("case_params_from_autopilot() - Done RX - ending at Millis:"); Serial.println(millis());


        // see what we got (any magic nums still in place?) and maybe timesatmp it.





        seconds_since_last_mavlink_req = 0;  // reset the counter
    }
    else
    {
        #ifdef MAVLINK_DEBUG 
            //debugPrintln("ase_params_from_autopilot() - NOT NOW");
        #endif
    }

}