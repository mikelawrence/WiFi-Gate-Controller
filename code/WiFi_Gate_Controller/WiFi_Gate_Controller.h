/*
  WiFi Gate Controller Project wide defines

  Copyright (c) 2019 Mike Lawrence

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef WIFI_GATE_CONTROLLER_H
#define WIFI_GATE_CONTROLLER_H

/******************************************************************
 * Build defines
 ******************************************************************/
// Enable Watchdog Timer
#define ENABLE_WATCHDOG
// Enable OTA updates
#define ENABLE_OTA_UPDATES
// Enable Serial on USB
#define ENABLE_SERIAL
// Enable Low Power Mode on WiFi
//#define ENABLE_WIFI_LOW_POWER
// Current Version
#define VERSION                   "0.5"

/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "Front Gate"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             50
// output contact time in milliseconds
#define CONTACT_TIME              500
// output contact dead time between contact closures in milliseconds
#define CONTACT_DEADBAND_TIME     100
// temperature measurement time in milliseconds, Must be greater than 5 seconds
#define TEMP_RATE                 1*60*1000

/******************************************************************
 * Home Assistant MQTT Defines
 ******************************************************************/
#define HASS_PREFIX               "homeassistant"
#define HASS_GATE_NAME            "front_gate"

// HASS defines below here should not be modified
#define HASS_AVAIL_TOPIC          HASS_PREFIX "/cover/" HASS_GATE_NAME "/avail"
#define HASS_PAYLOAD_AVAIL        "online"
#define HASS_PAYLOAD_NOT_AVAIL    "offline"

#define HASS_GATE_CONFIG_TOPIC    HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/config"
#define HASS_GATE_STATE_TOPIC     HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/state"
#define HASS_GATE_COMMAND_TOPIC   HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/set"
#define HASS_GATE_CONFIG          "{ \"name\": \"" BOARD_NAME "\", \"cmd_t\": \"" HASS_GATE_COMMAND_TOPIC \
                                  "\", \"stat_t\": \"" HASS_GATE_STATE_TOPIC "\", \"qos\": 1, \"retain\": false" \
                                  ", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"
                                  
#define HASS_TEMP_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_GATE_NAME "/temperature/config"
#define HASS_TEMP_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_GATE_NAME "/temperature/state"
#define HASS_TEMP_CONFIG          "{ \"name\": \"" BOARD_NAME " Temperature\", \"stat_t\": \"" HASS_TEMP_STATE_TOPIC \
                                  "\", \"unit_of_meas\": \"°C\", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"
                                  
#define HASS_RSSI_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_GATE_NAME "/rssi/config"
#define HASS_RSSI_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_GATE_NAME "/rssi/state"
#define HASS_RSSI_CONFIG          "{ \"name\": \"" BOARD_NAME " RSSI\", \"stat_t\": \"" HASS_RSSI_STATE_TOPIC \
                                  "\", \"unit_of_meas\": \"dBm\", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"
                                  
#define HASS_STATUS_CONFIG_TOPIC  HASS_PREFIX "/sensor/" HASS_GATE_NAME "/status/config"
#define HASS_STATUS_STATE_TOPIC   HASS_PREFIX "/sensor/" HASS_GATE_NAME "/status/state"
#define HASS_STATUS_CONFIG        "{ \"name\": \"" BOARD_NAME " Status\", \"stat_t\": \"" HASS_STATUS_STATE_TOPIC \
                                  "\", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"

/******************************************************************
 * Board Defines
 ******************************************************************/
// Output defines
#define NUMBER_OUTPUTS            4
#define OPEN_BUTTON               0
#define CLOSE_BUTTON              1
#define TOGGLE_BUTTON             2
#define UNUSED_BUTTON             3
// Input defines
#define NUMBER_INPUTS             8
#define OPEN_BUTTON_SENSE         0
#define CLOSE_BUTTON_SENSE        1
#define UNUSED_BUTTON_SENSE       2
#define TOGGLE_BUTTON_SENSE       3
#define OPEN_LIMIT_SENSE          4
#define CLOSE_LIMIT_SENSE         5
#define MAILBOX_SENSE             6
#define DROPBOX_SENSE             7
// LED
#define BOARD_LED                 LED_BUILTIN

// Logging/Printing defines
#ifdef ENABLE_SERIAL
#define Print(...)                Serial.print(__VA_ARGS__)
#define Println(...)              Serial.println(__VA_ARGS__)
#else
#define Print(...)
#define Println(...)
#endif

#endif // WIFI_GATE_CONTROLLER_H
