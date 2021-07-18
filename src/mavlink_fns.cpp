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
 * Request Data from Pixhawk - Pixhawk will not send any data until you request it.
 *============================*/
void mavlink_request_datastream()
{
  debugPrintln("mavlink_request_datastream() - Executing");
  delay(2000);    // just to give me time to see the message :)
  uint8_t _system_id = 255;      // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;     // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1;    // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
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

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)

  Serial1.write(buf, len); //Write data to serial port

} // END - mavlink_request_datastream()

/*============================
 * mavlink_receive()
 *
 * function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
 *============================*/
void mavlink_receive()
{
  //debugPrintln("mavlink_receive() - Executing");
  mavlink_message_t msg;
  mavlink_status_t status;
  //debugPrintln("char?");
  while (Serial1.available())   // xxx - I should prob put a time limiter on this WHILE, I think the only reason
                                // it is not hogging the CPU is because the AutoPilot cube only sends msgs each 
                                // second, and then pauses I think, is why the WHILE breaks out.
  {
    uint8_t c = Serial1.read();
    //debugPrintln("got char");
    // add new char to what we have so far and see of we have a full Mavlink msg yet
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) // if we do then lets process it
    {
      // debugs to show the msg # of the ones I'm seeing but not interested in.
      //debugPrint("#");
      //debugPrintInt(msg.msgid);
      //debugPrint("  ");
      //Decode new message from autopilot
      switch (msg.msgid)
      {
      //handle heartbeat message
      case MAVLINK_MSG_ID_HEARTBEAT:
      {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        debugPrint("\nMillis:");
        debugPrintInt(millis());
        debugPrint("  HEARTBEAT");
        Serial.print("  Flight Mode: (10 is auto):");
        Serial.print(hb.custom_mode);
        Serial.print("  Type:");
        Serial.print(hb.type);
        Serial.print("  Autopilot:");
        Serial.print(hb.autopilot);
        Serial.print("  Base Mode:");
        Serial.print(hb.base_mode);
        Serial.print("  System Status:");
        Serial.print(hb.system_status);
        Serial.print("  Mavlink Version:");
        Serial.println(hb.mavlink_version);

        // Save things I'm interested in to FeatherMx data structure for use later.
        myFeatherSettings.CUSTOMMODE = hb.custom_mode;
        myFeatherSettings.SYSTEMSTATUS = hb.system_status;

        break;
      }

      case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        //debugPrintln("Its a GPS RAW");
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(&msg, &packet);
        

        debugPrint("\nMillis:");
        debugPrintInt(millis());
        debugPrint("  GPS_RAW_INT");
        Serial.print("  Time:");
        time_t t = packet.time_usec / 1000000;  // time from GPS in mavlink is uint64_t in microseconds
                                                // so I divide by 1,000,000 to get it in seconds
                                                // which is what the hour(), minute() etc functions expect
        Serial.print(hour(t));
        Serial.print(":");
        Serial.print(minute(t));
        Serial.print(":");
        Serial.print(second(t));        
        Serial.print("  Fix Type:");
        Serial.print(packet.fix_type);
        Serial.print("  Latitude:");
        Serial.print(packet.lat);
        Serial.print("  Longitude:");
        Serial.print(packet.lon);
        Serial.print("  Ground Speed:");
        Serial.print(packet.vel);
        Serial.print("  CoG (deg):");
        Serial.print(packet.cog);
        Serial.print("  Sats Visible:");
        Serial.println(packet.satellites_visible);

        // Save things I'm interested in to FeatherMx data structure for use later.
        myFeatherSettings.GPSTIMESTAMP = packet.time_usec;
        myFeatherSettings.GPSHOUR = year(t);
        myFeatherSettings.GPSHOUR = month(t);
        myFeatherSettings.GPSHOUR = day(t);
        myFeatherSettings.GPSHOUR = hour(t);
        myFeatherSettings.GPSMIN = minute(t);
        myFeatherSettings.GPSHOUR = second(t);
        myFeatherSettings.GPSHOUR = 0;  // I'm not sure how to extract millis from t yet.
        myFeatherSettings.FIX = packet.fix_type;
        myFeatherSettings.LAT = packet.lat;
        myFeatherSettings.LON = packet.lon;
        myFeatherSettings.SPEED = packet.vel;
        myFeatherSettings.HEAD = packet.cog;
        myFeatherSettings.SATS = packet.satellites_visible;

        break;
      }

      // ************************************************************************************************
      // DEFAULT - should not happen, but programing it defensively
      default:
        //Serial.println("we hit the default: in mavlink packet decode switch");
        break;

      } // END - of msg decoder switch

      //delay(5000);
    }   // END - of IF we have a full mavlink packet
  }     // END - of while (Serial1.available())
} // END - MavLink_receive()
