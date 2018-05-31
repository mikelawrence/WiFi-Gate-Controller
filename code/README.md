# WiFi Gate Controller Sketch
The WiFi Gate Controller Sketch is written in C. It interfaces with a US Automation Gate Controller and will detect gate position and open/close the gate. The sketch is a discoverable cover and sensor for [Home Assistant](https://home-assistant.io/), an open-source home automation platform running on Python. [MQTT](http://mqtt.org/), a machine-to-machine (M2M)/"Internet of Things" connectivity protocol, is the basis of communication with Home Assistant.

## Status
WiFi is working nicely. Home Assistant detects and properly controls all MQTT devices: Gate Cover, Temperature Sensor, RSSI Sensor, and Status Sensor.

The biggest issue at this time is occasionally the Watchdog Timer resets the sketch. This can be seen via Home Assistant when the Gate Status Sensor contains 'Reset'. This reset occurs almost daily when "ENABLE_SERIAL" is defined and less frequently when not defined.

# Setup
## Sketch Setup
There are a few build defines that control how the sketch is built.

```c
/******************************************************************
 * Build defines
 ******************************************************************/
// Enable Watchdog Timer
#define ENABLE_WATCHDOG
// Enable OTA updates
//#define ENABLE_OTA_UPDATES
// Enable Serial on USB
//#define ENABLE_SERIAL
// Enable Low Power Mode on WiFi
#define ENABLE_WIFI_LOW_POWER
// Current Version
#define VERSION                   "0.1"
```

* When "ENABLE_WATCHDOG" is defined the Watchdog Timer is enabled with a 16 second timeout. This will reset the ARM processor is something goes bad.
* When "ENABLE_OTA_UPDATES" is defined the sketch can be updated using Arduino's Over-The-Air Update capability.
* When "ENABLE_SERIAL" is defined then status information is sent out the serial connection which in this case is the USB port. Otherwise the serial port won't even be enabled.
* When "ENABLE_WIFI_LOW_POWER" is defined the WINC1500 module is set to Low Power Mode. This can drop the current requirements of the module to a third or even less. Low Power Mode reduces the transmit frequency to the beacon interval and may also cause the module to hang occasionally.

The arduino_secrets.h file is not included on Github. You must create and edit it to meet your configuration.

```c
/******************************************************************
 * arduino_secrets.h
 ******************************************************************/
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
```

The rest of the sketch settings are C defines in the [WiFi Gate Controller Sketch](WiFi_Gate_Controller/WiFi_Gate_Controller.ino).

```c
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
```
* "BOARD_NAME" names the board so multiple instances of this board can on the WiFi network at the same time. It is used is several places including Home Assistant Name, OTA Name, and MQTT Client ID. This is a string.
* "DEBOUNCE_TIME" is how long to ignore contact changes on the input before accepting them as valid. This is an integer and the units are milliseconds.
* "CONTACT_TIME" is how long an output contact should be held active before being released. This is an integer and the units are milliseconds.
* "CONTACT_DEADBAND_TIME" is how long after a output contact has been released before another output contact may be activated. Any "OPEN" or "CLOSE" MQTT commands are ignored for CONTACT_TIME + CONTACT_DEADBAND_TIME milliseconds. This is an integer and the units are milliseconds.
* "TEMP_RATE" is how often the temperature should be sampled and updated via MQTT.  This is an integer and the units are milliseconds. The time must be greater than 5000 milliseconds.
* "HASS_PREFIX" is the Home Assistant MQTT Discovery Prefix as defined in your system. This is a string.
* "HASS_GATE_NAME" is used in the MQTT topics to identify the cover and sensor to Home Assistant. Home Assistant calls this the node id. It is a string and must not contain special characters including a space.

## Home Assistant Setup
The sketch is setup to enable MQTT Discovery on Home Assistant. If you don't want to use discovery here is the configuration of the gate in Home Assistant. Note the 'Front Gate' you see in the example yaml is the "BOARD_NAME" which is defined in [WiFi Gate Controller Sketch](WiFi_Gate_Controller/WiFi_Gate_Controller.ino) and 'front_gate' is "HASS_GATE_NAME". The WiFi Gate Controller uses the cover platform for Home Assistant ([MQTT Cover](https://www.home-assistant.io/components/cover.mqtt/)). The other sensors use the [MQTT Sensor](https://home-assistant.io/components/sensor.mqtt/) platform.

```yaml
# Example configuration.yaml entry
cover:
  # Front Gate cover
  - platform: mqtt_json
    name: "Front Gate"
    state_topic: "hass/cover/front_gate/gate/state"
    command_topic: "hass/cover/front_gate/gate/set"
    qos: 1
    retain: false

sensor:
  # Front Gate temperature
  - platform: mqtt
    name: "Front Gate Temperature"
    state_topic: "hass/cover/front_gate/temperature/state"
    unit_of_measurement: "°C"
  # Front Gate RSSI
  - platform: mqtt
    name: "Front Gate RSSI"
    state_topic: "hass/cover/front_gate/rssi/state"
    unit_of_measurement: "dBm"
  # Front Gate Status
  - platform: mqtt
    name: "Front Gate Status"
    state_topic: "hass/cover/front_gate/status/state"
```

Once the board is running Home Assistant should automatically pick up the cover and sensors with MQTT Discovery. Debugging Home Assistant problems is a bit of a stretch for this guide but here are a couple of hints.

* Make sure you have MQTT installed. If you use HASS.IO goto the HASS.IO configuration and install the Mosquitto Broker.
* Make sure you have MQTT discovery enabled. See [MQTT Discovery](https://home-assistant.io/docs/mqtt/discovery/).
* Make sure your MQTT discovery prefix matches the HASS_PREFIX in the [WiFi Gate Controller Sketch](WiFi_Gate_Controller/WiFi_Gate_Controller.ino).

I use HASS.IO with the Mosquitto Broker add-on installed and my configuration for MQTT is as follows...
```yaml
mqtt:
  broker: core-mosquitto
  discovery: true
  discovery_prefix: hass

```
