/*
 * mavlink_fns.ino
 * 
 * mavlink specific functions
 * 
 * 
 */

#include "global.h"

/*============================
 * mavlink_request_datastream()
 *
 * Request Data from Pixhawk - Pixhawk will "sometimes" not send any data until you request it.
 *============================*/
void mavlink_request_datastream()
{
  debugPrintln("mavlink_request_datastream() - Executing");
  // source and dest MAVLink addressing info.
  uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
  uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
  uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
  uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

  uint8_t _req_stream_id = MAV_DATA_STREAM_RAW_SENSORS; // MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1;           //1 = start, 0 = stop

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, // Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, // Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, // Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack and send the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
  Serial1.write(buf, len); //Write data to serial port

} // END - mavlink_request_datastream()


/*============================
 * mavlink_request_datastream()
 *
 * Request Data from Pixhawk - Pixhawk will "sometimes" not send any data until you request it.
 *============================*/
void mavlink_unrequest_datastream()
{
  debugPrintln("mavlink_unrequest_datastream() - Executing");
  // source and dest MAVLink addressing info.
  uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
  uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
  uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
  uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

  uint8_t _req_stream_id = MAV_DATA_STREAM_RAW_SENSORS; // MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 0;           //1 = start, 0 = stop

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, // Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, // Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, // Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack and send the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
  Serial1.write(buf, len); //Write data to serial port

} // END - mavlink_unrequest_datastream()



/*============================
 * mavlink_fmx_send_heartbeat_to_ap()
 *
 * Sends the correctly formatted MAVLink HEARTBEAT msg from FMX to AP
 * https://mavlink.io/en/messages/common.html#HEARTBEAT
 *============================*/
void mavlink_fmx_send_heartbeat_to_ap()
{
  debugPrintln("mavlink_fmx_send_heartbeat_to_ap() - Executing");
  // source and dest MAVLink addressing info.
  uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
  uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
  uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
  uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

  // components of the MAVLink HEARTBEAT message - https://mavlink.io/en/messages/common.html#HEARTBEAT
  uint8_t _hb_type      = MAV_TYPE_SURFACE_BOAT; // https://mavlink.io/en/messages/common.html#MAV_TYPE
  uint8_t _hb_autopilot = MAV_AUTOPILOT_INVALID; // https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT
  uint8_t _hb_basemode  = 0;  // https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
                              // None of the options looked appropriate so I have set to 0 for now.
  uint32_t _hb_custom_mode = 0; // None of the options looked appropriate so I have set to 0 for now.
  uint8_t _hb_system_status = MAV_STATE_ACTIVE; // https://mavlink.io/en/messages/common.html#MAV_STATE


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(_system_id, _component_id, &msg, _hb_type, _hb_autopilot, _hb_basemode, _hb_custom_mode, _hb_system_status);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
  Serial1.write(buf, len); //Write data to serial port byte by byte.

} // END - mavlink_fmx_send_heartbeat_to_ap()


/*============================
 * mavlink_request_streaming_params_from_ap()
 *
 * Asks the AP to stream certain MAVLink messages, at a regular interval, on the Serial
 * link to the FMX.
 * 
 * The FMX then uses other functions to then pull out what it wants from those streamed messages.
 * 
 *============================*/
void mavlink_request_streaming_params_from_ap()
{

    // Prep source and dest MAVLink addressing info, to be used in below actions.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    
    // Try to minimise other streaming message "noise" on the MAVLink port
    // ===================================================================
    // Note this is a bit out of place, but in a bit of a stateless way I'm just trying to, 
    // ensure that when the main code in this function is executed, I have recently squashed as much noise as possible.  The noise can be
    // due to Mission Planner telling the AP to stream various messages, or other left over crud from myself.  The point is though to quieten
    // as much of it down now as possible before moving on. This is a bit of a hack, but it does help me, particularly when troubleshooting.
    // I also try to minimise what the AutoPilot streams with 

    mavlink_unrequest_datastream(); // try to silence a common source (setoff by Mission Planner!!!!)


    // Request just the messages I am interested in and at a suitable interval
    // =======================================================================

    // ORIGINALLY I tried to use the MAV_CMD_REQUEST_MESSAGE method in a COMMAND_LONG (#76) (https://mavlink.io/en/messages/common.html#COMMAND_LONG) but I never
    // seemed to get a response, or even a COMMAND_ACK (#77) https://mavlink.io/en/messages/common.html#COMMAND_ACK back.
    // This was the method suggested in the ArduPilot doco at https://ardupilot.org/dev/docs/mavlink-requesting-data.html#using-request-message 
    // but it either did not work or I was doing something wrong???? 
    //
        // The code I tried to use....
        // Create components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
        //uint16_t _cl_command      = MAV_CMD_REQUEST_MESSAGE; // MAV_CMD_SET_MESSAGE_INTERVAL; // https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
        //uint8_t _cl_confirmation  = 0; // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
        //float   _cl_param1 = MAVLINK_MSG_ID_POWER_STATUS;   // MAVLink Message ID
        //float   _cl_param2 = 0; // Not used.
        //float   _cl_param3 = 0; // Not used, so set to zero.
        //float   _cl_param4 = 0; // Not used, so set to zero.
        //float   _cl_param5 = 0; // Not used, so set to zero.
        //float   _cl_param6 = 0; // Not used, so set to zero.
        //float   _cl_param7 = 0; // Not used, so set to zero.
    //
    // SO INSTEAD I fell back to using the streaming method (AP streams messages at regular intervals) and 
    // used MAV_CMD_SET_MESSAGE_INTERVAL method in a COMMAND_LONG (#76) (https://mavlink.io/en/messages/common.html#COMMAND_LONG) 
    // to ask the AP to stream the messages I wanted at a regular interval, and that seemed to work.  
    // The nominated messages would start flowing and I would also get the correct 
    // COMMAND_ACK (#77) https://mavlink.io/en/messages/common.html#COMMAND_ACK back from the AutoPilot, acknowledging the COMMAND_LONG's
    // I sent it to set those message intervals.

    // The code below places all those requests...

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

    // Request MAVLINK_MSG_ID_SYSTEM_TIME (#2) - https://mavlink.io/en/messages/common.html#SYSTEM_TIME
    _cl_param1 = MAVLINK_MSG_ID_SYSTEM_TIME;   // MAVLink Message ID
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

    // Request MAVLINK_MSG_ID_AUTOPILOT_VERSION (#148) - https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
    _cl_param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION;   // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len); //Write data to serial port byte by byte.
    delay(500);

} // END - request_streaming_params_from_ap()



void request_one_param_from_ap()
{
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
    uint16_t len;

    // MAV_TYPE
    // BATT_ARM_VOLT
    const char param_i_want[16] = "BATT_ARM_VOLT";

    debugPrintln("request_one_param_from_ap() - START");

    mavlink_msg_param_request_read_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want, -1);

    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len); //Write data to serial port byte by byte.
    
    debugPrintln("request_one_param_from_ap() - END");

}






/*============================
 * mavlink_receive()
 *
 * Function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino.
 * Params we want are then stored in myFeatherMxSettings.AP_xx ready for subsequent use/sharing on.
 * 
 * This function is called for two reasons;
 * i) To receive messages the AP is sending because in my case_params_from_autopilot() function we requested the AP stream 
 *    them at regular intervals OR
 * ii) It could be to look for a specific response to a specific request from the FMX to the AP. For example when I set a Mission.
 * 
 *============================*/
void mavlink_receive()
{
  //debugPrintln("mavlink_receive() - Executing");
  mavlink_message_t msg;
  mavlink_status_t status;
  bool gotFullMsg = false;
  //debugPrintln("char?");
  while (Serial1.available() && !gotFullMsg)   // xxx - I should prob put a time limiter on this WHILE, I think the only reason
                                // it is not hogging the CPU is because the AutoPilot cube only sends msgs each 
                                // second, and then pauses I think, is why the WHILE breaks out.
    {
    uint8_t c = Serial1.read();
    //debugPrintln("got char");
    // add new char to what we have so far and see of we have a full Mavlink msg yet
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) // if we do then bust out of this char collecting loop.
      gotFullMsg = true;
    } 
    
  // At this point we either have a Full Msg OR there were no more chars available on the MAVLink serial link.


  if (gotFullMsg) // if we do then lets process it
    {
    #ifdef MAVLINK_DEBUG
      // debugs to show the msg # of the ones I'm seeing but not interested in.
      debugPrint("mavlink_receive() - MSG RCVD -");
      debugPrint(" magic:");debugPrintInt(msg.magic);
      debugPrint(" seq:");debugPrintInt(msg.seq);
      debugPrint(" src sysid:");debugPrintInt(msg.sysid);
      debugPrint(" src compid:");debugPrintInt(msg.compid);
      debugPrint(" msgid#:");debugPrintInt(msg.msgid);
    #endif
    //Decode new message from autopilot
    switch (msg.msgid)
      {
      
      //============================
      case MAVLINK_MSG_ID_HEARTBEAT:
      {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        #ifdef MAVLINK_DEBUG
          debugPrint("=HEARTBEAT");
          Serial.print(" FlightMode:");
          Serial.print(hb.custom_mode);
          Serial.print(" Type:");
          Serial.print(hb.type);
          Serial.print(" Autopilot:");
          Serial.print(hb.autopilot);
          Serial.print(" BaseMode:");
          Serial.print(hb.base_mode);
          Serial.print(" CustomMode:");
          Serial.print(hb.custom_mode);
          Serial.print(" SystemStatus:");
          Serial.print(hb.system_status);
          Serial.print(" MavlinkVersion:");
          Serial.print(hb.mavlink_version);
        #endif

        seconds_since_last_mavlink_heartbeat_rx = 0;  // reset this timer as we just got a HEARTBEAT from the AP.

        // Save things I'm interested in to FeatherMx data structure for use later.
        // Note, because the GCS and even the ADSB sub controller in the Cube baseboard are seperate MAVLink "Components", they emit
        // their own HEARTBEAT msgs. SO below I need to check that I ONLY copy data from the AUtoPilot HEARTBEATS, and not the GCS
        // or ADSB node HEARTBEATS.
        if (hb.type == MAV_TYPE_SURFACE_BOAT)
        {
          myFeatherMxSettings.AP_BASEMODE = hb.base_mode;
          myFeatherMxSettings.AP_CUSTOMMODE = hb.custom_mode;
          myFeatherMxSettings.AP_SYSTEMSTATUS = hb.system_status;
        }
        break;
      }

      //============================
      case MAVLINK_MSG_ID_PARAM_VALUE:    // https://mavlink.io/en/messages/common.html#PARAM_VALUE
      {
        mavlink_param_value_t param_value;
        mavlink_msg_param_value_decode(&msg, &param_value);

        #ifdef MAVLINK_DEBUG
          debugPrint("=PARAM_VALUE");
          Serial.print(" param_id:");
          Serial.print(param_value.param_id);
          Serial.print(" param_value:");
          Serial.print(param_value.param_value);
          Serial.print(" param_type:");
          Serial.print(param_value.param_type);
          Serial.print(" param_count");
          Serial.print(param_value.param_count);
          Serial.print(" param_index");
          Serial.print(param_value.param_index);
        #endif

        // Save things I'm interested in to FeatherMx data structure for use later.

        // if (hb.type == MAV_TYPE_SURFACE_BOAT)
        // {
        //   myFeatherMxSettings.AP_BASEMODE = hb.base_mode;
        //   myFeatherMxSettings.AP_CUSTOMMODE = hb.custom_mode;
        //   myFeatherMxSettings.AP_SYSTEMSTATUS = hb.system_status;
        // }

        break;
      }




      //============================
      case MAVLINK_MSG_ID_GPS_RAW_INT:    // https://mavlink.io/en/messages/common.html#GPS_RAW_INT
      {
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(&msg, &packet);
        time_t t = packet.time_usec / 1000000;  // time from GPS in mavlink is uint64_t in microseconds
                                                // so I divide by 1,000,000 to get it in seconds
                                                // which is what the hour(), minute() etc functions expect
        #ifdef MAVLINK_DEBUG
          debugPrint("=GPS_RAW_INT");
          Serial.print(" Time:");
          Serial.print(hour(t));
          Serial.print("h:");
          Serial.print(minute(t));
          Serial.print("m:");
          Serial.print(second(t));        
          Serial.print("s FixType:");
          Serial.print(packet.fix_type);
          Serial.print(" Latitude:");
          Serial.print(packet.lat);
          Serial.print(" Longitude:");
          Serial.print(packet.lon);
          Serial.print(" GroundSpeed:");
          Serial.print(packet.vel);
          Serial.print(" CoG(deg):");
          Serial.print(packet.cog);
          Serial.print(" SatsVisible:");
          Serial.print(packet.satellites_visible);
        #endif

        // Save things I'm interested in to FeatherMx data structure for use later.
        myFeatherMxSettings.AP_VEL = packet.vel;
        myFeatherMxSettings.AP_COG = packet.cog;
        myFeatherMxSettings.AP_SATS = packet.satellites_visible;
        break;
      }

      //============================
      case MAVLINK_MSG_ID_HWSTATUS:   // https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS
        {
          mavlink_hwstatus_t packet;
          mavlink_msg_hwstatus_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=HWSTATUS");
            debugPrint(" Vcc:");
            Serial.print(packet.Vcc);
            debugPrint("mV I2Cerr:");
            Serial.print(packet.I2Cerr);
            debugPrint("errors");
          #endif

          break;
        }

      //============================
      case MAVLINK_MSG_ID_POWER_STATUS:   // https://mavlink.io/en/messages/common.html#POWER_STATUS
        {
          mavlink_power_status_t packet;
          mavlink_msg_power_status_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=POWER_STATUS");
            debugPrint(" Vcc:");
            Serial.print(packet.Vcc);
            debugPrint("mV Vservo:");
            Serial.print(packet.Vservo);
            debugPrint("mV flags:");
            Serial.print(packet.flags);
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_VCC = packet.Vcc;
          myFeatherMxSettings.AP_VSERVO = packet.Vservo;
          myFeatherMxSettings.AP_POWERFLAGS = packet.flags;
        
          break;
        }

      //============================
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:   // https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
        {
          mavlink_global_position_int_t packet;
          mavlink_msg_global_position_int_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=GLOBAL_POSITION_INT");
            debugPrint(" TimeSinceBoot:");Serial.print(packet.time_boot_ms);
            debugPrint("mS LAT:");Serial.print(packet.lat);
            debugPrint("degE7 LON:");Serial.print(packet.lon);
            debugPrint("degE7 MSL_ALT:");Serial.print(packet.alt);
            debugPrint("mm REL_ALT:");Serial.print(packet.relative_alt);
            debugPrint("mm VelX:");Serial.print(packet.vx);
            debugPrint("cm/s VelY:");Serial.print(packet.vy);
            debugPrint("cm/s VelZ:");Serial.print(packet.vz);
            debugPrint("cm/s Heading:");Serial.print(packet.hdg);
            debugPrint("cdeg");
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_POSITIONTIMESTAMP = packet.time_boot_ms;
          myFeatherMxSettings.AP_LAT = packet.lat;
          myFeatherMxSettings.AP_LON = packet.lon;
          myFeatherMxSettings.AP_VX = packet.vx;
          myFeatherMxSettings.AP_VY = packet.vy;
          myFeatherMxSettings.AP_HDG = packet.hdg;

          break;
        }

      //============================
      case MAVLINK_MSG_ID_SYS_STATUS:   // https://mavlink.io/en/messages/common.html#SYS_STATUS
        {
          mavlink_sys_status_t packet;
          mavlink_msg_sys_status_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=SYS_STATUS");
            debugPrint(" onboard_control_sensors_present:");
            Serial.print(packet.onboard_control_sensors_present);
            debugPrint(" onboard_control_sensors_enabled:");
            Serial.print(packet.onboard_control_sensors_enabled);
            debugPrint(" onboard_control_sensors_health:");
            Serial.print(packet.onboard_control_sensors_health);
            debugPrint(" load:");
            Serial.print(packet.load);
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_SENSORSPRESENT = packet.onboard_control_sensors_present;
          myFeatherMxSettings.AP_SENSORSENABLED = packet.onboard_control_sensors_enabled;
          myFeatherMxSettings.AP_SENSORSHEALTH = packet.onboard_control_sensors_health;
          myFeatherMxSettings.AP_LOAD = packet.load;
        
          break;
        }

      //============================
      case MAVLINK_MSG_ID_SYSTEM_TIME:   // https://mavlink.io/en/messages/common.html#SYSTEM_TIME
        {
          mavlink_system_time_t packet;
          mavlink_msg_system_time_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=SYSTEM_TIME");
            debugPrint(" time_unix_usec:");
            Serial.print("I can't print uint64_t's here");
            debugPrint(" time_boot_ms:");
            Serial.print(packet.time_boot_ms);
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_TIMEUNIXUSEC = packet.time_unix_usec;
          myFeatherMxSettings.AP_TIMEBOOTMS = packet.time_boot_ms;
        
          break;
        }

      //============================
      case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:   // https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
        {
          mavlink_nav_controller_output_t packet;
          mavlink_msg_nav_controller_output_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=NAV_CONTROLLER_OUTPUT");
            debugPrint(" nav_bearing:");
            Serial.print(packet.nav_bearing);
            debugPrint(" target_bearing:");
            Serial.print(packet.target_bearing);
            debugPrint(" wp_dist:");
            Serial.print(packet.wp_dist);
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_NAVBEARING = packet.nav_bearing;
          myFeatherMxSettings.AP_TARGETBEARING = packet.target_bearing;
          myFeatherMxSettings.AP_WPDIST = packet.wp_dist;

          break;
        }

      //============================
      case MAVLINK_MSG_ID_BATTERY_STATUS:   // https://mavlink.io/en/messages/common.html#BATTERY_STATUS
        {
          mavlink_battery_status_t packet;
          mavlink_msg_battery_status_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=BATTERY_STATUS");
            debugPrint(" voltages[0]:");
            Serial.print(packet.voltages[0]);
            debugPrint(" voltages[1]:");
            Serial.print(packet.voltages[1]);
            debugPrint(" current_battery:");
            Serial.print(packet.current_battery);
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_VOLTAGES[0] = packet.voltages[0];
          myFeatherMxSettings.AP_VOLTAGES[1] = packet.voltages[1];
          myFeatherMxSettings.AP_CURRENTBATTERY = packet.current_battery;
        
          break;
        }

      //============================
      case MAVLINK_MSG_ID_AUTOPILOT_VERSION:   // https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
        {
          mavlink_autopilot_version_t packet;
          mavlink_msg_autopilot_version_decode(&msg, &packet);

          #ifdef MAVLINK_DEBUG
            debugPrint("=AUTOPILOT_VERSION");
            debugPrint(" vendor_id:");
            Serial.print(packet.vendor_id);
            debugPrint(" product_id:");
            Serial.print(packet.product_id);
          #endif

          // Save things I'm interested in to FeatherMx data structure for use later.
          myFeatherMxSettings.AP_VENDORID = packet.vendor_id;
          myFeatherMxSettings.AP_PRODUCTID = packet.product_id;
        
          break;
        }


      //============================
      // DEFAULT - should not happen, but programing it defensively
      default:
        //Serial.println("we hit the default: in mavlink packet decode switch");
        break;

      } // END - of msg decoder switch

      #ifdef MAVLINK_DEBUG
        debugPrintln("");
      #endif
    }   // END - of IF we have a full mavlink packet lets process it

} // END - MavLink_receive()
