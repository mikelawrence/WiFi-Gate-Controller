/*
  WiFi Gate Controller 
  
  This code interfaces a remote gate opener (US Automation) using a custom 
  SAMD/ATWINC1500C board. The board is compatible with an Arduino MKR1000.
  
  Built with Arduino IDE 1.8.9
  
  The following libraries must be installed using Library Manager:
  
    WiFi101 version 0.16.0 by Arduino
      WINC1501 Model B firmware version 19.6.1
    WiFi101OTA version 1.0.2 by Arduino
    MQTT version 2.4.3 by Joel Gaehwiler
    OneWire version 2.3.4 by Paul Stoffregen and many others
    DallasTemperature version 3.8.0 by Miles Burton and others
  
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
#define ENABLE_OTA_UPDATES
// Enable Serial on USB
#define ENABLE_SERIAL
// Enable Low Power Mode on WiFi
//#define ENABLE_WIFI_LOW_POWER
// Current Version
#define VERSION                   "0.4"

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
// pin number array for the four outputs on this board (pins are D0, D1, D2, D3)
const int8_t output_pins[NUMBER_OUTPUTS] = {0, 1, 2, 3};
// pin number array for the eight inputs on this board (pins are A0, A1, A2, A3, D4, D5, A4, D7)
const int8_t input_pins[NUMBER_INPUTS] = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, 4, 5, PIN_A4, 7};
// Enumeration for gate state which also keeps track of gate opening or closing
enum GateStateEnum {GS_UNKNOWN, GS_RESET, GS_ERROR, GS_CLOSED, GS_OPEN, GS_CLOSING, GS_OPENING}; 
// time in milliseconds that a WiFi connection is unresponsive before reconnecting
#define WIFI_CONNECTION_RETRY_TIME 5*60*1000
// time in milliseconds between MQTT connection attempts
#define MQTT_CONNECTION_DELAY_TIME 10*1000

// Logging/Printing defines
#ifdef ENABLE_SERIAL
#define Print(...)                Serial.print(__VA_ARGS__)
#define Println(...)              Serial.println(__VA_ARGS__)
#else
#define Print(...)
#define Println(...)
#endif

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
int8_t   inputState[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1, -1, -1};
// last Gate open/closed state true/false
int8_t   lastGateOpenState = -1;
// when true a reset occurred recently
bool     resetOccurred = true;
// when true WiFi was recently disconnected from the network
bool     wifiDisconnectOccurred = true;
// when true WiFi was recently connected to the network
bool     wifiConnectOccurred = false;

/******************************************************************
 * Reset the watchdog timer
 ******************************************************************/
inline void watchdogReset(void) {
  #ifdef ENABLE_WATCHDOG
  if (!WDT->STATUS.bit.SYNCBUSY)                // Check if the WDT registers are synchronized
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;        // Clear the watchdog timer
  #endif
}

/******************************************************************
 * Verify/ Make WiFi and MQTT connections
 ******************************************************************/
bool connect() {
  static uint8_t  lastMQTTRetryCount = 0;
  static uint32_t lastMQTTRetryTime = millis() - 10*1000;
  static uint32_t lastDisconnectTime;
  byte mac[6];
  IPAddress ip;
  int32_t status = WiFi.status();
  
  // Reset the watchdog every time this function is run
  watchdogReset();
  
  if (status != WL_CONNECTED) {
    // Wifi is disconnected
    wifiDisconnectOccurred = true;                    // we were disconnected
    if (abs(millis() - lastDisconnectTime) > 4*WIFI_CONNECTION_RETRY_TIME) {
      // something went wrong with lastDisconnectTime, reset
      lastDisconnectTime = millis() - WIFI_CONNECTION_RETRY_TIME - 10;
    }
    if (millis() - lastDisconnectTime > WIFI_CONNECTION_RETRY_TIME) {
      // it's been too long since we were last connected
      WiFi.end();                                     // turn off WiFi module
      // Turn on WINC1500 WiFi module and connect to network again
      WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
      // Reconnect in another 15 minutes
      lastDisconnectTime = millis();
      Println("Reconnecting to SSID: " SECRET_SSID "...");
    }
    return false;                                     // return with not connected
  }
  
  // update last time we were disconnected to now because WiFi is connected
  lastDisconnectTime = millis();
  
  if (wifiDisconnectOccurred && (status == WL_CONNECTED)) {
    // WiFi just connected
    Println("Connected to SSID: " SECRET_SSID);
    
    // we have detected that we just connected
    wifiDisconnectOccurred = false;                   // so we won't print network stats until next reconnect
    wifiConnectOccurred = true;                       // so MQTT publishing will know that Wifi just connected
    
    // Enable WiFi Low Power Mode
    #ifdef ENABLE_WIFI_LOW_POWER
    WiFi.lowPowerMode();
    Println("  Low Power Mode enabled");
    #endif
    
    #ifdef ENABLE_SERIAL
    // Display MAC Address
    WiFi.macAddress(mac);
    Print("  MAC Address: ");
    for (int i = 5; i != 0; i--) {
      if (mac[i] < 16) Print("0");
      Print(mac[i], HEX);
      Print(":");
    }
    if (mac[0] < 16) Print("0");
    Println(mac[0], HEX);
    
    // Display IP Address
    ip = WiFi.localIP();
    Print("  IP Address: ");
    Println(ip);
    #endif
    
    #ifdef ENABLE_OTA_UPDATES
    // start the WiFi OTA library with internal based storage
    WiFiOTA.begin(BOARD_NAME, OTA_PASSWORD, InternalStorage);
    Println("WiFi OTA updates enabled");
    #endif
  }
  
  // WiFi is connected, see if MQTT is connected
  if (!mqtt.connected()) {
    if (++lastMQTTRetryCount >= WIFI_CONNECTION_RETRY_TIME/MQTT_CONNECTION_DELAY_TIME) {
      // it's been too long since we had an MQTT connection, turn off WiFi module
      WiFi.end(); 
      // Turn on WINC1500 WiFi module and connect to network again
      WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
      lastMQTTRetryTime = millis();                   // restart MQTT retry time
      lastMQTTRetryCount = 0;                         // MQTT retry count will need to start over
      return false;                                   // we are not connected
    }
    if (abs(millis() - lastMQTTRetryTime) > 4*MQTT_CONNECTION_DELAY_TIME) {
      // something went wrong with lastMQTTRetryTime, reset
      lastMQTTRetryTime = millis() - MQTT_CONNECTION_DELAY_TIME - 1;
    }
    // we are not currently connected to MQTT Server
    if (millis() - lastMQTTRetryTime <= MQTT_CONNECTION_DELAY_TIME) {
      // not time to retry MQTT connection
      return false;
    }
    // time to retry server connection
    Println("Connecting to MQTT Server:" MQTT_SERVER "...");
    if (mqtt.connect(BOARD_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      // successfully connected to MQTT server
      Println("Success");
    } else {
      // failed to connect to MQTT server
      Println("Failed");
      ++lastMQTTRetryCount;
      lastMQTTRetryTime = millis();
      return false;
    }
        
    // Subscribe to Home Assistant command topic
    if (!mqtt.subscribe(HASS_GATE_COMMAND_TOPIC)) {
      Println("  Failed to subscribe '" HASS_GATE_COMMAND_TOPIC "' MQTT topic");
    } else {
      Println("  Subscribed to '" HASS_GATE_COMMAND_TOPIC "' MQTT topic");
    }
    
    // Publish Home Assistant gate config topic
    if (!mqtt.publish(HASS_GATE_CONFIG_TOPIC, HASS_GATE_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_GATE_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_GATE_CONFIG_TOPIC "' MQTT topic, '" HASS_GATE_CONFIG "' topic value");
    }
    
    // Publish Home Assistant temperature config topic
    if (!mqtt.publish(HASS_TEMP_CONFIG_TOPIC, HASS_TEMP_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic, '" HASS_TEMP_CONFIG "' topic value");
    }
    
    // Publish Home Assistant RSSI config topic
    if (!mqtt.publish(HASS_RSSI_CONFIG_TOPIC, HASS_RSSI_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic, '" HASS_RSSI_CONFIG "' topic value");
    }
    
    // Publish Home Assistant status config topic
    if (!mqtt.publish(HASS_STATUS_CONFIG_TOPIC, HASS_STATUS_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic, '" HASS_STATUS_CONFIG "' topic value");
    }
    
    if (resetOccurred) {                              // Hardware reset occurred
      // reset just recently occurred
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset Hardware", true, 1)) {
        Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
      }
    } else if (wifiConnectOccurred) {                 // WiFi Connected
      // Wifi just connected
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset WiFi Connect", true, 1)) {
        Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
      }
/*    } else {
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset MQTT Connect", true, 1)) {
        // since no reset or WiFi connect occurred then it was a MQTT Connect
        Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset MQTT Connect' topic value");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset MQTT Connect' topic value");
      } */
    }
    
    // wait a bit before updating HASS_STATUS_STATE_TOPIC again
    delay(1000);
    
    // clear the connect reason flags
    resetOccurred = false;
    wifiConnectOccurred = false;
    
    Println("WiFi Gate Controller is ready!\n");

    // we had to reconnect to MQTT Broker make sure at least one false is returned
    return false;
  }
  
  // be ready for next MQTT retry time
  lastMQTTRetryTime = millis();
  lastMQTTRetryCount = 0;
    
  // if we got here then we were successfull
  return true;
}

/******************************************************************
 * MQTT subscribed message received
 ******************************************************************/
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
        Println("Accepted 'OPEN' MQTT Command");
      } else if (strcmp(payload, "CLOSE") == 0) {     // time to CLOSE the gate
        digitalWrite(output_pins[CLOSE_BUTTON], HIGH);// make CLOSE Button active
        outputState[CLOSE_BUTTON] = HIGH;             // indicate to loop() that CLOSE Button output is active
        lastOutputTime[CLOSE_BUTTON] = millis();      // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[CLOSE_BUTTON]; // start a timer for deadband timeout
        Println("Accepted 'CLOSE' MQTT Command");
     } else if (strcmp(payload, "STOP") == 0) {      // time to TOGGLE the gate
        digitalWrite(output_pins[TOGGLE_BUTTON], HIGH); // make TOGGLE Button active
        outputState[TOGGLE_BUTTON] = HIGH;            // indicate to loop() that TOGGLE Button output is active
        lastOutputTime[TOGGLE_BUTTON] = millis();     // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[TOGGLE_BUTTON]; // start a timer for deadband timeout
        Println("Accepted 'STOP' MQTT Command");
      }
    } else {
      // another output is in progress so we ignore this command
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Ignored MQTT Command", true, 1)) {
        Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Ignored MQTT Command' topic value");
      }
      Print("Ignored '");
      Print(payload);
      Println("' MQTT Command");
    }
  }
}

/******************************************************************
 * Standard Arduino setup function
 ******************************************************************/
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
  Println("\nWiFi Gate Controller: " BOARD_NAME);
  Println("  Software Version: " VERSION);
  
  // Start up the Dallas Temperature library 
  sensors.begin();
  // see that we have at least one temperature sensor
  #ifdef ENABLE_SERIAL
  if (sensors.getDeviceCount() == 0) {
      Println("No 1-Wire temperature sensors found.");
  } else {
    DeviceAddress deviceAddress;
    sensors.getAddress(deviceAddress, 0);
    Print("Found ");
    switch (deviceAddress[0]) {
      case 0x10:
        Print("DS18S20");
        break;
      case 0x28:
        Print("DS18B20");
        break;
      case 0x22:
        Print("DS1822");
        break;
      case 0x3B:
        Print("DS1825");
        break;
      default:
        Print("Unknown");
        break;
    }
    Print(" temperature sensor: 0x");
    for (int i = 0; i < 8; i++)
    {
      if (deviceAddress[i] < 16) Print("0");
      Print(deviceAddress[i], HEX);
    }
    Println("");
  }
  #endif
  
  // Configuration based on board type
  #if   defined(ARDUINO_SAMD_FEATHER_M0)
  // Adafruit Feather M0 WINC1500 is sometimes used for testing
  Println("Arduino Board Type: Adafruit Feather M0");
  WiFi.setPins(8,7,4,2);                              // Feather M0 needs the WiFi pins redefined
  #elif defined(ARDUINO_SAMD_MKR1000)
  // The WiFi Gate Controller board emulates a MRK1000 board
  Println("Arduino Board Type: Arduino MKR1000");
  #else
  // Other boards can work but may require WiFi pin redefinition
  Println("Arduino Board Type: Unknown");
  #endif
  
  // WiFi setup
  if (WiFi.status() == WL_NO_SHIELD) {                // check for the presence of the WINC1500
    Println("WINC1500 not present! Nothing can be done!");
    // don't continue:
    while (true);
  }
  
  Println("WINC1500 Detected");
  
  #ifdef ENABLE_SERIAL
  // Display Firmware Version
  String fv = WiFi.firmwareVersion();
  Print("  Firmware Version: ");
  Println(fv);
  Print("  Library Version: ");
  Println(WIFI_FIRMWARE_LATEST_MODEL_B);
  #endif
        
  // Turn on WINC1500 WiFi module and connect to network now
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
  
  // MQTT setup
  mqtt.setOptions(65, true, 5000);                    // keep Alive, Clean Session, Timeout
  mqtt.begin(MQTT_SERVER, MQTT_SERVERPORT, net);
  mqtt.onMessageAdvanced(messageReceived);
  
  #ifdef ENABLE_WATCHDOG
  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 256Hz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(6) |              // Divide the 32.768kHz clock source by divisor 32
                                                      //   where 2^(6 + 1): 32.768kHz/128=256Hz
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

  REG_WDT_CONFIG = WDT_CONFIG_PER_16K;                // Set the WDT reset timeout to 16384 clock cycles or 64s seconds
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  REG_WDT_CTRL = WDT_CTRL_ENABLE;                     // Enable the WDT in normal mode
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  #endif

  // check connections
  connect();
}

/******************************************************************
 * Standard Arduino loop function
 ******************************************************************/
void loop() {
  static uint32_t lastDebounceTime[NUMBER_INPUTS] = {0, 0, 0, 0, 0, 0, 0, 0};
  static int8_t   lastDebounceInput[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static uint32_t lastTempUpdate = 0 - TEMP_RATE;     // used to measure the temp at a defined rate
  static uint32_t lastTempRequest = 0 - TEMP_RATE;    // used to delay the time between temp request and temp read
  static GateStateEnum curGateState = GS_RESET;       // current gate state for state machine
  static GateStateEnum nextGateState = GS_UNKNOWN;    // next gate state for state machine
  char            tempStr[16];                        // temporary string
  float           lastTemp;                           // last measured temperature
  uint8_t         inputReading;                       // last input read
  bool            inputStabilized = true;             // when true inputs are stabilized and are ready to be read
  uint8_t         oneInputActive;                     // when true one of the command inputs is active
  
  // Reset the watchdog every time loop() is called
  //watchdogReset();
  
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

  // turn LED on if any of the command inputs are high
  oneInputActive = false;
  for(int i = 0; i < 3; i++) {
    if (inputState[i] == HIGH) {
      oneInputActive = true;
      break;
    }
  }
  if (oneInputActive) {
    digitalWrite(BOARD_LED, HIGH);
  } else {
    digitalWrite(BOARD_LED, LOW);
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
            Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, '' topic value");
          } else {
            Println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, '' topic value");
          }
          break;
        case GS_CLOSED:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Closed", true, 1)) {
            publishFailed = true;                     // publish failed
            Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closed' topic value");
          } else {
            Println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closed' topic value");
          }
          break;
        case GS_OPEN:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Open", true, 1)) {
            publishFailed = true;                     // publish failed
            Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Open' topic value");
          } else {
            Println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Open' topic value");
          }
          break;
        case GS_CLOSING:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Closing", true, 1)) {
            publishFailed = true;                     // publish failed
            Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closing' topic value");
          } else {
            Println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Closing' topic value");
          }
          break;
        case GS_OPENING:
          if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Gate Opening", true, 1)) {
            publishFailed = true;                     // publish failed
            Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Opening' topic value");
          } else {
            Println("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Gate Opening' topic value");
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
          Print("Failed to publish '" HASS_GATE_STATE_TOPIC "' MQTT topic, '");
          Print(tempStr);
          Println("' topic value");
        } else {
          Print("Published '" HASS_GATE_STATE_TOPIC "' MQTT topic, '");
          Print(tempStr);
          Println("' topic value");
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
        if (lastTempUpdate > millis()) {
          lastTempUpdate = millis();                  // don't want to play catch up
        }
        lastTemp = sensors.getTempCByIndex(0);        // retrieve the temperature from the first temp sensor
        dtostrf(lastTemp, 1, 1, tempStr);             // convert temperature to string
        // publish the just read temperture
        if (!mqtt.publish(HASS_TEMP_STATE_TOPIC, tempStr, false, 1)) {
          Print("Failed to publish '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
          Print(tempStr);
          Println("' topic value");
        } else {
          Print("Published '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
          Print(tempStr);
          Println("' topic value");
        }
        itoa(WiFi.RSSI(), tempStr, 10);
        // publish the RSSI too
        if (!mqtt.publish(HASS_RSSI_STATE_TOPIC, tempStr, false, 1)) {
          Print("Failed to publish '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
          Print(tempStr);
          Println("' topic value");
        } else {
          Print("Published '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
          Print(tempStr);
          Println("' topic value");
        }
      }
    }
  }
}
