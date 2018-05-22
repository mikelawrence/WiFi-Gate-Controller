/*
  WiFi Gate Controller 
  
  This code interfaces a remote gate opener (US Automation) using a custom 
  SAMD/ATWINC1500C board. The board is compatible with an Arduino MKR1000.

  The following libraries must be installed using Library Manager:
  
    WiFi101 by Arduino
    WiFiOTA by Arduino
    MQTT by Joel Gaehwiler
    OneWire by Paul Stoffregen and many others
    DallasTemperature by Miles Burton and others
  
  Copyright (c) 2018 Mike Lawrence

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
// Modify ../WiFi101/src/bsp/include/nm_bsp_internal.h library file
// so that CONF_PERIPH is defined. This will enabled LEDS from WiFi Module
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <MQTT.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <avr/dtostrf.h>
#include "arduino_secrets.h"
/******************************************************************
 * Definitions in arduino_secrets.h
 ******************************************************************/
/*
#define SECRET_SSID             "SSID of your WiFi network"
#define SECRET_PASSWORD         "Password for your WIFi network"
// IP address of MQTT server, host name may work but I've never tried
#define MQTT_SERVER             "192.168.0.230"
#define MQTT_SERVERPORT         1883
// MQTT user name and password leave as empty string if not used
#define MQTT_USERNAME           ""
#define MQTT_PASSWORD           ""
// Over-The-Air Update password if used
#define OTA_PASSWORD            "password"
*/

/******************************************************************
 * Build defines
 ******************************************************************/
// Enable Watchdog Timer
#define ENABLE_WATCHDOG
// Enable OTA updates
//#define ENABLE_OTA_UPDATES
// Enable Serial on USB
//#define ENABLE_SERIAL
// Current Version
#define VERSION                   "0.1"

/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "Front Gate"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             25
// output contact time in milliseconds
#define CONTACT_TIME              500
// output contact dead time between contact closures in milliseconds
#define CONTACT_DEADBAND_TIME     100
// temperature measurement time in milliseconds, Must be greater than 5 seconds
#define TEMP_RATE                 5*60*1000

/******************************************************************
 * Home Assistant MQTT Defines
 ******************************************************************/
#define HASS_PREFIX               "hass"
#define HASS_GATE_NAME            "front_gate"
// HASS defines below here should not be modified
#define HASS_GATE_CONFIG_TOPIC    HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/config"
#define HASS_GATE_STATE_TOPIC     HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/state"
#define HASS_GATE_COMMAND_TOPIC   HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/set"
#define HASS_GATE_CONFIG          "{ \"name\": \"" BOARD_NAME "\", \"command_topic\": \"" HASS_GATE_COMMAND_TOPIC \
                                  "\", \"state_topic\": \"" HASS_GATE_STATE_TOPIC "\", \"qos\": 1, \"retain\": false }"
#define HASS_TEMP_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_GATE_NAME "/temperature/config"
#define HASS_TEMP_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_GATE_NAME "/temperature/state"
#define HASS_TEMP_CONFIG          "{ \"name\": \"" BOARD_NAME " Temperature\", \"state_topic\": \"" HASS_TEMP_STATE_TOPIC \
                                  "\", \"unit_of_measurement\": \"°C\" }"
#define HASS_RSSI_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_GATE_NAME "/rssi/config"
#define HASS_RSSI_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_GATE_NAME "/rssi/state"
#define HASS_RSSI_CONFIG          "{ \"name\": \"" BOARD_NAME " RSSI\", \"state_topic\": \"" HASS_RSSI_STATE_TOPIC \
                                  "\", \"unit_of_measurement\": \"dBm\" }"
#define HASS_STATUS_CONFIG_TOPIC  HASS_PREFIX "/sensor/" HASS_GATE_NAME "/status/config"
#define HASS_STATUS_STATE_TOPIC   HASS_PREFIX "/sensor/" HASS_GATE_NAME "/status/state"
#define HASS_STATUS_CONFIG        "{ \"name\": \"" BOARD_NAME " Status\", \"state_topic\": \"" HASS_STATUS_STATE_TOPIC "\" }"

/******************************************************************
 * Board Defines
 ******************************************************************/
// Output defines
#define NUMBER_OUTPUTS          4
#define OPEN_BUTTON             0
#define CLOSE_BUTTON            1
#define TOGGLE_BUTTON           2
#define UNUSED_BUTTON           3
// Input defines
#define NUMBER_INPUTS           8
#define OPEN_BUTTON_SENSE       0
#define CLOSE_BUTTON_SENSE      1
#define UNUSED_BUTTON_SENSE     2
#define TOGGLE_BUTTON_SENSE     3
#define OPEN_LIMIT_SENSE        4
#define CLOSE_LIMIT_SENSE       5
#define MAILBOX_SENSE           6
#define DROPBOX_SENSE           7
// LED
#define BOARD_LED               LED_BUILTIN
// pin number array for the four outputs on this board (pins are D0, D1, D2, D3)
const int8_t output_pins[NUMBER_OUTPUTS] = {0, 1, 2, 3};
// pin number array for the eight inputs on this board (pins are A0, A1, A2, A3, D4, D5, A4, D7)
const int8_t input_pins[NUMBER_INPUTS] = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, 4, 5, PIN_A4, 7};
// Enumeration for gate state which also keeps track of gate opening or closing
enum GateStateEnum {GS_UNKNOWN, GS_RESET, GS_ERROR, GS_CLOSED, GS_OPEN, GS_CLOSING, GS_OPENING}; 

/******************************************************************
 * Global Variables
 ******************************************************************/
// Network
WiFiClient net;
// MQTT CLient
MQTTClient mqtt(1024);
// oneWire instance to communicate with any 1-Wire devices  
OneWire oneWire(PIN_A5); 
// Dallas Temperature measurement . 
DallasTemperature sensors(&oneWire);

// used to keep track of time to keep outputs active
uint32_t lastOutputTime[NUMBER_OUTPUTS] = {0, 0, 0, 0};
// current output state, arranged in array by input_pins
uint8_t  outputState[NUMBER_OUTPUTS] = {LOW, LOW, LOW, LOW};
// output in progress including deadband bewteen contact closures
uint8_t  outputDeadband = HIGH;
// used to keep track of deadband time between concact closures
uint32_t lastOutputDeadbandTime = 0;
// current input state, arranged in array by input_pins
uint8_t  inputState[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1, -1, -1};
// last Gate open/closed state true/false
int8_t   lastGateOpenState = -1;
// when true a reset occurred recently
bool     resetOccurred = true;
// when true WiFi was recently disconnected from the network
bool     wifiDisconnectOccurred = true;
// when true WiFi was recently connected to the network
bool     wifiConnectOccurred = false;

// Arduino setup function
void setup() {
  // Serial setup
  #ifdef ENABLE_SERIAL
  //while (!Serial);                                  // wait until serial monitor is open
  Serial.begin(115200);
  #endif
  
  // configure LED output
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LOW);                       // LED pin is an output and currently off
  
  // configure outputs
  for (int i = 0; i < NUMBER_OUTPUTS; i++) {
    pinMode(output_pins[i], OUTPUT);                  // set mode to output
    digitalWrite(output_pins[i], LOW);                // set output to LOW
  }
  
  // configure inputs (all active high, all push/pull driven)
  for (int i = 0; i < NUMBER_INPUTS; i++) {
    pinMode(input_pins[i], INPUT);                    // set mode to input
  }
  
  // takes a bit for the USB to come up
  #ifdef ENABLE_SERIAL
  delay(2000);
  #endif
  
  // Announce who we are and software
  #ifdef ENABLE_SERIAL
  Serial.println("\nWiFi Gate Controller: " BOARD_NAME);
  Serial.println("  Software Version: " VERSION);
  #endif
  
  // Start up the Dallas Temperature library 
  sensors.begin();
  // see that we have at least one temperature sensor
  #ifdef ENABLE_SERIAL
  if (sensors.getDeviceCount() == 0) {
      Serial.println("No 1-Wire temperature sensors found.");
  } else {
    DeviceAddress deviceAddress;
    sensors.getAddress(deviceAddress, 0);
    Serial.print("Found ");
    switch (deviceAddress[0]) {
      case 0x10:
        Serial.print("DS18S20");
        break;
      case 0x28:
        Serial.print("DS18B20");
        break;
      case 0x22:
        Serial.print("DS1822");
        break;
      case 0x3B:
        Serial.print("DS1825");
        break;
      default:
        Serial.print("Unknown");
        break;
    }
    Serial.print(" temperature sensor: 0x");
    for (int i = 0; i < 8; i++)
    {
      if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
    }
    Serial.println("");
  }
  #endif
  
  // Set the WiFi pins (Needed by Adafruit Feather M0 WINC1500)
  //WiFi.setPins(8,7,4,2);
  
  // WiFi setup
  if (WiFi.status() == WL_NO_SHIELD) {                // check for the presence of the WINC1500
    #ifdef ENABLE_SERIAL
    Serial.println("WINC1500 not present! Nothing can be done!");
    #endif
    // don't continue:
    while (true);
  }
  #ifdef ENABLE_SERIAL
  Serial.println("WINC1500 Detected");
  
  // Display Firmware Version
  String fv = WiFi.firmwareVersion();
  Serial.print("  Firmware Version: ");
  Serial.println(fv);
  Serial.print("  Library Version: ");
  Serial.println(WIFI_FIRMWARE_LATEST_MODEL_B);
  #endif
        
  // Turn on WINC1500 WiFi module and connect to network now
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);

  // MQTT setup
  mqtt.setOptions(15*60, true, 5000);                 // keep Alive, Clean Session, Timeout
  mqtt.begin(MQTT_SERVER, MQTT_SERVERPORT, net);
  mqtt.onMessageAdvanced(messageReceived);
  
  #ifdef ENABLE_WATCHDOG
  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |              // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);                // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |            // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |               // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |             // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K |     // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);              // Select GCLK2         
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |             // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |         // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;             // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  REG_WDT_CONFIG = 0xBu;                              // Set the WDT reset timeout to 16384 clock cycles or 16s seconds
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  REG_WDT_CTRL = WDT_CTRL_ENABLE;                     // Enable the WDT in normal mode
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  #endif

  // check connections
  connect();
}

// Arduino loop function
void loop() {
  static uint32_t lastDebounceTime[NUMBER_INPUTS] = {0, 0, 0, 0, 0, 0, 0, 0};
  static uint8_t  lastDebounceInput[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static uint32_t lastTempUpdate = 0 - TEMP_RATE;     // used to measure the temp at a defined rate
  static uint32_t lastTempRequest = 0 - TEMP_RATE;    // used to delay the time between temp request and temp read
  static GateStateEnum curGateState = GS_RESET;       // current gate state for state machine
  static GateStateEnum nextGateState = GS_UNKNOWN;    // next gate state for state machine
  char            tempStr[16];                        // temporary string
  float           lastTemp;                           // last measured temperature
  uint8_t         inputReading;                       // last input read
  bool            inputStabilized = true;             // when true inputs are stabilized and are ready to be read
  
  // Reset the watchdog with every loop to make sure the sketch keeps running.
  watchdogReset();
  
  #ifdef ENABLE_OTA_UPDATES
  // check for WiFi OTA updates
  WiFiOTA.poll();
  #endif
  
  // MQTT client loop call
  mqtt.loop();
  
  // debounce input
  for (int i = 0; i < NUMBER_INPUTS; i++) {
    // read the current input state
    inputReading = digitalRead(input_pins[i]);
    // update last debounce time if the state has changed
    if (inputReading != lastDebounceInput[i]) {
      // reset the debounce timer
      lastDebounceTime[i] = millis();
    }
    // if debounce time has elapsed then the input is stable
    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_TIME) {
      // debouced finished but update only if changed
      if (inputState[i] != inputReading) {
        // input actually changed, not updated with the same value
        inputState[i] = inputReading;                 // update input
      }
    }
    // last is now current
    lastDebounceInput[i] = inputReading;
  }
  
  // handle deactivating outputs
  for(int i = 0; i < NUMBER_OUTPUTS; i++) {
    if (outputState[i] == HIGH) {
      // output is currently active
      if ((millis() - lastOutputTime[i]) > CONTACT_TIME) {
        // output has been active long enough
        outputState[i] = LOW;                         // deactive output
        digitalWrite(output_pins[i], LOW);            // make output inactive
        digitalWrite(BOARD_LED, LOW);                 // deactivate visual indication
      }
    }
  }
  
  // handle deadband between output activations
  if (outputDeadband == HIGH) {
    // deadband is currently active
    if ((millis() - lastOutputDeadbandTime) > (CONTACT_TIME + CONTACT_DEADBAND_TIME)) {
      // deadband has been active long enough
      outputDeadband = LOW;                           // allow another output to go active
    }
  }
  
  // inputs must have stabilized before they are read for the first time
  for(int i = 0; i < NUMBER_INPUTS; i++) {
    // check input to see if it is still default
    if (inputState[i] == -1) {
      inputStabilized = false;
      break;
    }
  }
  
  // if inputs are stabilized then check inputs as normal
  if (inputStabilized) {
    // handle Gate State Machine
    switch (curGateState) {
      case GS_RESET:
        if ((inputState[CLOSE_LIMIT_SENSE] == HIGH) && 
            (inputState[OPEN_LIMIT_SENSE] == HIGH)) {
          // Error condition because gate is OPEN and CLOSED at the same time
          nextGateState = GS_ERROR;                   // switch to Error State
        } else if (inputState[CLOSE_LIMIT_SENSE] == HIGH) {
          // Gate is now Closed
          nextGateState = GS_CLOSED;                  // switch to Closed State
        } else if (inputState[OPEN_LIMIT_SENSE] == HIGH) {
          // Gate is now Open all the way
          nextGateState = GS_OPEN;                    // switch to Open State
        }
        break;
      case GS_ERROR:
        if ((inputState[CLOSE_LIMIT_SENSE] == HIGH) && 
            (inputState[OPEN_LIMIT_SENSE] == LOW)) {
          // gate is now closed 
          nextGateState = GS_CLOSED;                  // switch to Closed State
        } else if ((inputState[CLOSE_LIMIT_SENSE] == LOW) && 
                   (inputState[OPEN_LIMIT_SENSE] == HIGH)) {
          // gate is now closed 
          nextGateState = GS_OPEN;                    // switch to Open State
        }
        break;
      case GS_CLOSED:
        if ((inputState[CLOSE_LIMIT_SENSE] == HIGH) && 
            (inputState[OPEN_LIMIT_SENSE] == HIGH)) {
          // Error condition because gate is OPEN and CLOSED at the same time
          nextGateState = GS_RESET;                   // switch to Error State
        } else if (inputState[CLOSE_LIMIT_SENSE] == LOW) {
          // Gate is now Opening
          nextGateState = GS_OPENING;                 // switch to Opening State
        } 
        break;
      case GS_OPEN:
        if ((inputState[CLOSE_LIMIT_SENSE] == HIGH) && 
            (inputState[OPEN_LIMIT_SENSE] == HIGH)) {
          // Error condition because gate is OPEN and CLOSED at the same time
          nextGateState = GS_RESET;                   // switch to Error State
        } else if (inputState[OPEN_LIMIT_SENSE] == LOW) {
          // Gate is now Closing
          nextGateState = GS_CLOSING;                 // switch to Closing State
        }
        break;
      case GS_CLOSING:
        if ((inputState[CLOSE_LIMIT_SENSE] == HIGH) && 
            (inputState[OPEN_LIMIT_SENSE] == HIGH)) {
          // Error condition because gate is OPEN and CLOSED at the same time
          nextGateState = GS_RESET;                   // switch to Error State
        } else if (inputState[CLOSE_LIMIT_SENSE] == HIGH) {
          // Gate is now Closed
          nextGateState = GS_CLOSED;                  // switch to Closed State
        } else if (inputState[OPEN_LIMIT_SENSE] == HIGH) {
          // Gate is now Open
          nextGateState = GS_OPEN;                    // switch to Open State
        }
        break;
      case GS_OPENING:
        if ((inputState[CLOSE_LIMIT_SENSE] == HIGH) && 
            (inputState[OPEN_LIMIT_SENSE] == HIGH)) {
          // Error condition because gate is OPEN and CLOSED at the same time
          nextGateState = GS_RESET;                   // switch to Error State
        } else if (inputState[OPEN_LIMIT_SENSE] == HIGH) {
          // Gate is now Open
          nextGateState = GS_OPEN;                    // switch to Open State
        } else if (inputState[CLOSE_LIMIT_SENSE] == HIGH) {
          // Gate is now Closed
          nextGateState = GS_CLOSED;                  // switch to Closed State
        }
        break;
      default:
        // unknown state, force back to reset state
        nextGateState = GS_RESET;
        break;
    }
  }
    
  // Check connections
  if (!connect()) {
    // we are not currently connected, ignore rest of loop to prevent MQTT publishing
    // set Gate States so next loop will detect current Gate position
    //   and if connected will force a publish of Gate State
    curGateState = GS_RESET;
    nextGateState = GS_UNKNOWN;
    return;
  }

  // if inputs are stabilized then check inputs as normal
  if (inputStabilized) {
    bool publishFailed = false;                       // defualt is successful publish
      
    // inputs are stable, handle change of Gate Status
    if (nextGateState != curGateState) {
      switch (nextGateState) {
        case GS_RESET:
          break;
        case GS_ERROR:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Error", true, 1)) {
            publishFailed = true;                     // publish failed
            #ifdef ENABLE_SERIAL
            Serial.println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, '' topic value");
            #endif
          } else {
            #ifdef ENABLE_SERIAL
            Serial.println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, '' topic value");
            #endif
          }
          break;
        case GS_CLOSED:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Closed", true, 1)) {
            publishFailed = true;                     // publish failed
            #ifdef ENABLE_SERIAL
            Serial.println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closed' topic value");
            #endif
          } else {
            #ifdef ENABLE_SERIAL
            Serial.println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closed' topic value");
            #endif
          }
          break;
        case GS_OPEN:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Open", true, 1)) {
            publishFailed = true;                     // publish failed
            #ifdef ENABLE_SERIAL
            Serial.println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Open' topic value");
            #endif
          } else {
            #ifdef ENABLE_SERIAL
            Serial.println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Open' topic value");
            #endif
          }
          break;
        case GS_CLOSING:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Closing", true, 1)) {
            publishFailed = true;                     // publish failed
            #ifdef ENABLE_SERIAL
            Serial.println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closing' topic value");
            #endif
          } else {
            #ifdef ENABLE_SERIAL
            Serial.println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closing' topic value");
            #endif
          }
          break;
        case GS_OPENING:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Opening", true, 1)) {
            publishFailed = true;                     // publish failed
            #ifdef ENABLE_SERIAL
            Serial.println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Opening' topic value");
            #endif
          } else {
            #ifdef ENABLE_SERIAL
            Serial.println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Opening' topic value");
            #endif
          }
          break;
        default:
          break;
      }
      
      // convert Current Gate State to open, closed, or error
      uint8_t curGateStateOpen = true;
      if (curGateState == GS_UNKNOWN) {
        // gate is in error
        curGateStateOpen = -1;
      } else if (curGateState == GS_RESET) {
        // gate is in error
        curGateStateOpen = -1;
      } else if (curGateState == GS_ERROR) {
        // gate is in error
        curGateStateOpen = -2;
      } else if (curGateState == GS_CLOSED) {
        // gate is closed
        curGateStateOpen = false;
      }
      
      // convert Next Gate State to open, closed, or error
      uint8_t nextGateStateOpen = true;
      if (nextGateState == GS_UNKNOWN) {
        // gate is in error
        nextGateStateOpen = -1;
      } else if (nextGateState == GS_RESET) {
        // gate is in error
        nextGateStateOpen = -1;
      } else if (nextGateState == GS_ERROR) {
        // gate is in error
        nextGateStateOpen = -2;
      } else if (nextGateState == GS_CLOSED) {
        // gate is closed
        nextGateStateOpen = false;
      }

      // handle change of gate state
      if (curGateStateOpen != nextGateStateOpen) {
        switch (nextGateStateOpen) {
          case true:
            strcpy(tempStr, "open");                  // gate is not closed
            break;
          case false:
            strcpy(tempStr, "closed");                // we are closed
            break;
          default:       
            strcpy(tempStr, "error");                 // gate is in error
            break;
        }
        // publish gate state change
        if (!mqtt.publish(HASS_GATE_STATE_TOPIC, tempStr, true, 1)) {
          publishFailed = true;                       // publish failed
          #ifdef ENABLE_SERIAL
          Serial.print("Failed to publish '" HASS_GATE_STATE_TOPIC "' MQTT topic, '");
          Serial.print(tempStr);
          Serial.println("' topic value");
          #endif
        } else {
          #ifdef ENABLE_SERIAL
          Serial.print("Published '" HASS_GATE_STATE_TOPIC "' MQTT topic, '");
          Serial.print(tempStr);
          Serial.println("' topic value");
          #endif
        }
      }

      // check for failure to publish states
      if (publishFailed) {
        // set Gate States so next loop will detect current Gate position
        //   and if connected will force a publish of Gate State
        curGateState = GS_RESET;
        nextGateState = GS_UNKNOWN;
      } else {
        // nextGateState now becomes current gate state
        curGateState = nextGateState;
      }
    }
  }
 
  // is it time to request a temperature?
  if (sensors.getDeviceCount() > 0) {    
    // at least one 1-wire temp device is available
    if ((millis() - lastTempUpdate) > TEMP_RATE) {
      // it has been long enough since last temperature was read
      if ((millis() - lastTempRequest) > 2*1000) {
        // elapsed time is greater than 5 seconds so this is the first check
        sensors.requestTemperatures();                // start temperature conversion
        lastTempRequest = millis();                   // start tracking conversion time
      } else if ((millis() - lastTempRequest) > 1000) {
        // conversion is complete
        lastTempUpdate += TEMP_RATE;                  // set timer for next time to start temperature conversion
        lastTemp = sensors.getTempCByIndex(0);        // retrieve the temperature from the first temp sensor
        dtostrf(lastTemp, 1, 1, tempStr);             // convert temperature to string
        // publish the just read temperture
        if (!mqtt.publish(HASS_TEMP_STATE_TOPIC, tempStr, false, 1)) {
          #ifdef ENABLE_SERIAL
          Serial.print("Failed to publish '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
          Serial.print(tempStr);
          Serial.println("' topic value");
          #endif
        } else {
          #ifdef ENABLE_SERIAL
          Serial.print("Published '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
          Serial.print(tempStr);
          Serial.println("' topic value");
          #endif
        }
        itoa(WiFi.RSSI(), tempStr, 10);
        // publish the RSSI too
        if (!mqtt.publish(HASS_RSSI_STATE_TOPIC, tempStr, false, 1)) {
          #ifdef ENABLE_SERIAL
          Serial.print("Failed to publish '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
          Serial.print(tempStr);
          Serial.println("' topic value");
          #endif
        } else {
          #ifdef ENABLE_SERIAL
          Serial.print("Published '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
          Serial.print(tempStr);
          Serial.println("' topic value");
          #endif
        }
      }
    }
  }
}

// Verify/Make connections
bool connect() {
  byte mac[6];
  IPAddress ip;

  if (WiFi.status() != WL_CONNECTED) {
    // Wifi is disconnected
    wifiDisconnectOccurred = true;                    // keep track of the fact we were disconnected
    return false;                                     // return with not connected
  }
  if (wifiDisconnectOccurred && (WiFi.status() == WL_CONNECTED)) {
    // WiFi is connected and previously we were disconnected
    #ifdef ENABLE_SERIAL
    Serial.println("Connected to SSID: " SECRET_SSID);
    #endif

    // we have detected that we just connected
    wifiDisconnectOccurred = false;                   // so we won't print network stats until next reconnect
    wifiConnectOccurred = true;                       // so MQTT publishing will know that Wifi just connected

    // Enable WiFi Low Power Mode
    WiFi.lowPowerMode();
    
    #ifdef ENABLE_SERIAL
    Serial.println("  Low Power Mode enabled");
    
    // Display MAC Address
    WiFi.macAddress(mac);
    Serial.print("  MAC Address: ");
    for (int i = 5; i != 0; i--) {
      if (mac[i] < 16) Serial.print("0");
      Serial.print(mac[i], HEX);
      Serial.print(":");
    }
    if (mac[0] < 16) Serial.print("0");
    Serial.println(mac[0], HEX);
    
    // Display IP Address
    ip = WiFi.localIP();
    Serial.print("  IP Address: ");
    Serial.println(ip);
    #endif

    #ifdef ENABLE_OTA_UPDATES
    // start the WiFi OTA library with internal based storage
    WiFiOTA.begin(BOARD_NAME, OTA_PASSWORD, InternalStorage);
    #ifdef ENABLE_SERIAL
    Serial.println("WiFi OTA updates enabled");
    #endif
    #endif
  } 
  
  if (!mqtt.connected()) {
    // we are not currently connected to MQTT Server
    #ifdef ENABLE_SERIAL
    Serial.print("Connecting to MQTT Server:" MQTT_SERVER "...");
    #endif
    if (mqtt.connect(BOARD_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      // successfully connected to MQTT server
      #ifdef ENABLE_SERIAL
      Serial.println("Success");
      #endif
    } else {
      // failed to connect to MQTT server
      #ifdef ENABLE_SERIAL
      Serial.println("Failed");
      printWiFiStatus(true);
      #endif
      return false;
    }
    
    // Subscribe to Home Assistant command topic
    if (!mqtt.subscribe(HASS_GATE_COMMAND_TOPIC)) {
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to subscribe '" HASS_GATE_COMMAND_TOPIC "' MQTT topic");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Subscribed to '" HASS_GATE_COMMAND_TOPIC "' MQTT topic");
      #endif
    }
    
    // Publish Home Assistant gate config topic
    if (!mqtt.publish(HASS_GATE_CONFIG_TOPIC, HASS_GATE_CONFIG, true, 1)) {
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to publish '" HASS_GATE_CONFIG_TOPIC "' MQTT topic");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Published '" HASS_GATE_CONFIG_TOPIC "' MQTT topic, '" HASS_GATE_CONFIG "' topic value");
      #endif
    }
    
    // Publish Home Assistant temperature config topic
    if (!mqtt.publish(HASS_TEMP_CONFIG_TOPIC, HASS_TEMP_CONFIG, true, 1)) {
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to publish '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Published '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic, '" HASS_TEMP_CONFIG "' topic value");
      #endif
    }
    
    // Publish Home Assistant RSSI config topic
    if (!mqtt.publish(HASS_RSSI_CONFIG_TOPIC, HASS_RSSI_CONFIG, true, 1)) {
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to publish '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Published '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic, '" HASS_RSSI_CONFIG "' topic value");
      #endif
    }
    
    // Publish Home Assistant status config topic
    if (!mqtt.publish(HASS_STATUS_CONFIG_TOPIC, HASS_STATUS_CONFIG, true, 1)) {
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to publish '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Published '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic, '" HASS_STATUS_CONFIG "' topic value");
      #endif
    }
    
    if (resetOccurred) {                              // Hardware reset occurred
      // reset just recently occurred
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset Hardware", true, 1)) {
        #ifdef ENABLE_SERIAL
        Serial.println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
        #endif
      } else {
        #ifdef ENABLE_SERIAL
        Serial.println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
        #endif
      }
    } else if (wifiConnectOccurred) {                 // WiFi Connected
      // Wifi just connected
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset WiFi Connect", true, 1)) {
        #ifdef ENABLE_SERIAL
        Serial.println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
        #endif
      } else {
        #ifdef ENABLE_SERIAL
        Serial.println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
        #endif
      }
    } else if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset MQTT Connect", true, 1)) {
      // since no reset or WiFi connect occurred then it was a MQTT Connect
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset MQTT Connect' topic value");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset MQTT Connect' topic value");
      #endif
    }
    
    // wait a bit before updating HASS_STATUS_STATE_TOPIC again
    delay(100);
    
    // clear the connect reason flags
    resetOccurred = false;
    wifiConnectOccurred = false;
    
    // running successfully
    if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Ready", true, 1)) {
      #ifdef ENABLE_SERIAL
      Serial.println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Ready' topic value");
      #endif
    } else {
      #ifdef ENABLE_SERIAL
      Serial.println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Ready' topic value");
      #endif
    }
    #ifdef ENABLE_SERIAL
    Serial.println("WiFi Gate Controller is ready!\n");
    #endif
  }

  // if we got here then we were successfull
  return true;
}

// MQTT subscribed message received
void messageReceived(MQTTClient *client, char topic[], char payload[], int payload_length) {
  if (strcmp(topic, HASS_GATE_COMMAND_TOPIC) == 0) {
    // gate command has been received
    if (outputDeadband == LOW) {
      // no other output is in progress
      if (strcmp(payload, "OPEN") == 0) {             // time to OPEN the gate
        digitalWrite(output_pins[OPEN_BUTTON], HIGH); // make OPEN Button active
        outputState[OPEN_BUTTON] = HIGH;              // indicate to loop() that OPEN Button output is active
        lastOutputTime[OPEN_BUTTON] = millis();       // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[OPEN_BUTTON]; // start a timer for deadband timeout
        digitalWrite(BOARD_LED, HIGH);                // visual indication that an output is active
        #ifdef ENABLE_SERIAL
        Serial.println("Accepted 'OPEN' MQTT Command");
        #endif
      } else if (strcmp(payload, "CLOSE") == 0) {     // time to CLOSE the gate
        digitalWrite(output_pins[CLOSE_BUTTON], HIGH);// make CLOSE Button active
        outputState[CLOSE_BUTTON] = HIGH;             // indicate to loop() that CLOSE Button output is active
        lastOutputTime[CLOSE_BUTTON] = millis();      // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[CLOSE_BUTTON]; // start a timer for deadband timeout
        digitalWrite(BOARD_LED, HIGH);                // visual indication that an output is active
        #ifdef ENABLE_SERIAL
        Serial.println("Accepted 'CLOSE' MQTT Command");
        #endif
     } else if (strcmp(payload, "STOP") == 0) {      // time to TOGGLE the gate
        digitalWrite(output_pins[TOGGLE_BUTTON], HIGH); // make TOGGLE Button active
        outputState[TOGGLE_BUTTON] = HIGH;            // indicate to loop() that TOGGLE Button output is active
        lastOutputTime[TOGGLE_BUTTON] = millis();     // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[TOGGLE_BUTTON]; // start a timer for deadband timeout
        digitalWrite(BOARD_LED, HIGH);                // visual indication that an output is active
        #ifdef ENABLE_SERIAL
        Serial.println("Accepted 'STOP' MQTT Command");
        #endif
      }
    } else {
      // another output is in progress so we ignore this command
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Ignored MQTT Command", true, 1)) {
        #ifdef ENABLE_SERIAL
        Serial.println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic");
        #endif
      } else {
        #ifdef ENABLE_SERIAL
        Serial.println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Ignored MQTT Command' topic value");
        #endif
      }
      #ifdef ENABLE_SERIAL
      Serial.print("Ignored '");
      Serial.print(payload);
      Serial.println("' MQTT Command");
      #endif
    }
  }
}

// Reset the watchdog timer
inline void watchdogReset(void) {
  #ifdef ENABLE_WATCHDOG
  if (!WDT->STATUS.bit.SYNCBUSY)                // Check if the WDT registers are synchronized
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;        // Clear the watchdog timer
  #endif
}
// Prints current WiFi Status and RSSI
void printWiFiStatus(bool withLF) {
  #ifdef ENABLE_SERIAL
  Serial.print("Status: ");
  switch (WiFi.status()) {
    case WL_NO_SHIELD: Serial.print("No Shield"); break;
    case WL_IDLE_STATUS: Serial.print("Idle Status"); break;
    case WL_NO_SSID_AVAIL: Serial.print("No SSID Available"); break;
    case WL_SCAN_COMPLETED: Serial.print("Scan Completed"); break;
    case WL_CONNECTED: Serial.print("Connected"); break;
    case WL_CONNECT_FAILED: Serial.print("Connect Failed"); break;
    case WL_CONNECTION_LOST: Serial.print("Connection Lost"); break;
    case WL_DISCONNECTED: Serial.print("Disconnected"); break;
  }
  Serial.print(", RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.print("dBm");
  if (withLF)
    Serial.println(".");
  else
    Serial.print(", ");
  #endif
}

