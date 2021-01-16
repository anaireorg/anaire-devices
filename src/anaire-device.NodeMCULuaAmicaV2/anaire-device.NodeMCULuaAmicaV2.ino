// Since 20201109 ANAIRE DEVICE CODE anaire.org anaire@anaire.org
//
// GNU General Public License v3.0
//
// Get CO2, temperature and humidity measurements and send them to the Anaire Cloud Application
// Show local measurements and provide local warning and alarm visual and sound indications
//
// Parts - Common platform:
//   Control board: AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
//   Buzzer: AZDelivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
//   Display: AZDelivery 0.91 inch OLED I2C Display 128 x 32 Pixels  https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display
//
// Parts - Sensors:
//   Option 1, better measurements precission: Sensirion SCD30 for CO2, temperature and humidity https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-co2/
//   Option 2, less cost: MHZ14A for CO2 http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf and AZ-Delivery DHT11 Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
//
// Arduino IDE Setup:
//   Install the usb to uart driver from https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
//   Start Arduino and open Preferences window.
//   Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
//   Open Boards Manager from Tools > Board menu and find esp8266 platform by esp8266 community and install the software for Arduino from a drop-down box.
//   Select "NodeMCU 1.0" board from Tools > Board menu after installation
//
// Install the following libraries in Arduino IDE:
//   WiFiEsp for WiFi https://github.com/bportaluri/WiFiEsp
//   DHTesp - to manage DHT11 or DHT22 temperature and humidity sensors https://github.com/beegee-tokyo/DHTesp
//   MHZ-Z19 Library from WifWaf https://github.com/WifWaf/MH-Z19 Library - to manage Winsen CO2 sensors - INSTALL FROM ZIP FILE with Sketch-> Include Library-> Add .ZIP library
//   Arduino Client for MQTT - for MQTT communications https://pubsubclient.knolleary.net/
//   ArduinoJson https://arduinojson.org/?utm_source=meta&utm_medium=library.properties
//   esp8266-oled-ssd1306 for oled display https://github.com/ThingPulse/esp8266-oled-ssd1306
//   paulvha SCD30 library https://github.com/paulvha/scd30 - INSTALL FROM ZIP FILE with Sketch-> Include Library-> Add .ZIP library
//   ESP_EEPROM - to save in EEPROM config values https://github.com/jwrw/ESP_EEPROM
//   WifiManager kentaylor - to create a captive portal to configure wifi https://github.com/kentaylor/WiFiManager
//   Double Reset detector - to detect double press of reset button and restart with the captive portal to configure Wifi https://github.com/datacute/DoubleResetDetector

// Design leads:
// - The ID and IP address are shown on OLED display during boot and after pressing the Flash button
// - Pressing the reset button (left of usb connector when facing it) twice in less than 10 seconds restarts the device in a captive web configuration portal, useful to configure local wifi settings
// - All other local config parameters (like name, thresholds, local alarm, etc.) are configured via the cloud app, after connecting
// - Pressing Flash button (right of the USB connector while facing it) toggles between activating/deactivating local alarms (visual and sound)
//   * Stops local alerting (both sound and blinking) until pressed again or until CO2 level decays below the warning threshold, deactivating therefore local alerting and reseting the local alerting system
//   * It also shows device ID and IP address after being pressed until a new CO2 measurement is displayed
// - Firmware update with latest fimrware available though cloud app
//   * Always using latest firmware on https://github.com/anaireorg/anaire-devices/blob/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin
// - Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector) is also connected to the external buzzer
//   * When CO2 Status is "ok" (below warning threshold) LED keeps ON and buzzer is off
//   * When CO2 Status is "warning" builtin LED and external buzzer alternate at a slow pace (WARNING_BLINK_PERIOD)
//   * When CO2 Status is "alarm" builtin LED and external buzzer alternate at fast pace (ALARM_BLINK_PERIOD)
//   This GPIO16 is also used when downloading SW to the nodemcu board: the external buzzer sounds while downloading to the board
// - SSD1306 OLED display is connected on GPIO4(D2) and GPIO2 (D4)
// - Two options for sensors:
//   * Either Sensirion SCD30 for CO2, temperature and humidity
//     * connected on pins 3V3, GND, GPIO14(D5) and GPIO12(D6), all consecutive on the NodeMCU LUA Amica V2 right hand side and coincident on the first 4 pins on SCD30
//   * Or MH-Z14A for CO2, and DHT11 for temnperature and humidity
//     * MHZ-14A CO2 sensor is connected through a software serial port using GPIO13(D7) and GPIO15(D8)
//     * DHT11 humidity and temperature sensor ins connected on GPIO 5 (D1)
// - The device is designed to work only with the CO2 sensor, so buzzer, DHT11 humidity and temperature sensor, and OLED display are optional
// - The device is designed to recover from Wifi, MQTT or sensors reading temporal failures
// - The web server is activated, therefore entering the IP on a browser allows to see the device measurements and thresholds.

String sw_version = "v1.20210116.adinadainadé";  // 

// CLOUD CONFIGURATION: remote app url
// CHANGE HERE if connecting to a different Anaire Cloud App
String cloud_server_address = "demo.anaire.org";     // server public IP address
int cloud_app_port = 30183;                          // cloud application port

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen
// i.e: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String anaire_device_id = String(ESP.getChipId(), HEX);   // HEX version, for easier match to mac address
String anaire_device_name = String(ESP.getChipId(), HEX); // By the default the name gets initialized to the device ID
int CO2ppm_warning_threshold = 700; // Warning threshold initial value
int CO2ppm_alarm_threshold = 1000;  // Alarm threshold initial value

// Save config values to EEPROM
#include <ESP_EEPROM.h>

// The neatest way to access variables stored in EEPROM is using a structure
struct MyEEPROMStruct {
  String anaire_device_name;
  int CO2ppm_warning_threshold;
  int CO2ppm_alarm_threshold;
  String cloud_server_address;
  int cloud_app_port;
  boolean local_alarm;
  boolean update_latest;
} eepromConfig;

// Control Loop: time between measurements
unsigned int control_loop_duration = 2000;  // 2 seconds
unsigned long control_loop_start;           // holds a timestamp for each control loop start
unsigned long lastReconnectAttempt = 0;     // MQTT reconnections

// CO2 Blinking period, used to reflect CO2 status on builtin led and buzzer
const int WARNING_BLINK_PERIOD = 1000; // 1 second
const int ALARM_BLINK_PERIOD = 100;    // 0.1 seconds

// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// WiFi
#include <ESP8266WiFi.h> // Wifi ESP8266
#include <ESP8266mDNS.h> // to be reached on anaire_device_id.local in the local network
WiFiClient wifi_client;
const int WIFI_CONNECT_TIMEOUT = 10000; // 10 seconds
int wifi_status = WL_IDLE_STATUS;
WiFiServer wifi_server(80); // to check if it is alive
String wifi_ssid = WiFi.SSID();                  // your network SSID (name)

// Double reset for wifi configuration
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/kentaylor/WiFiManager
#include <DoubleResetDetector.h>  //https://github.com/datacute/DoubleResetDetector

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// OLED ssd1306 screen
#include <Wire.h>
#include "SSD1306Wire.h"

// SCL and SDA pin connections
#define OLED_SCK_GPIO 4 // signal GPIO2 (D4)
#define OLED_SDA_GPIO 2 // signal GPIO0 (D3)

// for 128x32 displays:
SSD1306Wire display(0x3c, OLED_SDA_GPIO, OLED_SCK_GPIO, GEOMETRY_128_32);  // ADDRESS, SDA, SCL

// MQTT
#include <PubSubClient.h>
char mqtt_message[256];
String mqtt_send_topic = "measurement";
String mqtt_receive_topic = "config/" + anaire_device_id;  // config messages will be received in config/id
//String mqtt_message;
PubSubClient mqttClient(wifi_client);

//JSON
#include <ArduinoJson.h>

// Sensirion SCD CO2, temperature and humidity sensor
#define SCD30WIRE Wire
//#include "paulvha_SCD30.h"
#include "SparkFun_SCD30_Arduino_Library.h"                   //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#define SCD30_SCK_GPIO 14 // signal GPIO14 (D5)
#define SCD30_SDA_GPIO 12 // signal GPIO12 (D6)
SCD30 airSensor;
unsigned long SCD30_WARMING_TIME = 60000;                     // SCD30 CO2 sensor warming time: 60 seconds
unsigned long SCD30_CALIBRATION_TIME = 180000;                // SCD30 CO2 CALIBRATION TIME: 3 min = 180000 ms
//bool SCD30_AutoSelfCalibration = false;                       // SCD30 auto self calibration disabled
uint16_t SCD30_MEASUREMENT_INTERVAL = control_loop_duration;  // time between measurements
uint16_t SCD30_FORCED_CALIBRATION = 400;                      // SCD30 cero reference in a clean environment - Recommended 400
uint16_t SCD30_TEMPERATURE_OFFSET = 0;                        // SCD30 TEMPERATURE OFFSET: 5ºC - That is because of the proximity of temp sensor to NodeMCU board
uint16_t SCD30_ALTITUDE_COMPENSATION = 650;                   // Set to 650 meters, Madrid (Spain) mean altitude
#define SCD30_SERIAL_NUM_WORDS 3                    // added August 2020
// The longer serial number is 16 words / 32 bytes (means 48 bytes with CRC).
// Most I2C buffers are by default 32. Hence the length is kept to the
// 3 words = first 6 (equal to what is printed on the case).
// The additional information is for Senserion internal only.

// MHZ14A CO2 sensor: software serial port
#include "SoftwareSerial.h"                            // Remove if using HardwareSerial or non-uno compatabile device
#include "MHZ19.h"                                     // https://github.com/WifWaf/MH-Z19 Library
MHZ19 myMHZ19;
#define MHZ_BAUDRATE 9600                              // Native to the sensor (do not change)
const unsigned long MHZ14A_WARMING_TIME = 180000;      // MHZ14A CO2 sensor warming time: 3 minutes = 180000 ms
const unsigned long MHZ14A_SERIAL_TIMEOUT = 5000;      // MHZ14A CO2 serial start timeout: 5 seconds = 5000 ms
const unsigned long MHZ14A_CALIBRATION_TIME = 1200000; // MHZ14A CO2 CALIBRATION TIME: 20 min = 1200000 ms
#define swSerialRX_gpio 13
#define swSerialTX_gpio 15
//SoftwareSerial swSerial(swSerialRX_gpio, swSerialTX_gpio, false);
SoftwareSerial mySerial(swSerialRX_gpio, swSerialTX_gpio);
byte measurement_command[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // Command to get measurements from MHZ14A CO2 sensor
byte calibration_command[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // Command to calibrate MHZ14A CO2 sensor
char response_CO2[9]; // holds the received data from MHZ14A CO2 sensor
int response_CO2_high; // holds upper byte
int response_CO2_low;  // holds lower byte
int CO2ppm_value = 0;  // CO2 ppm measured value

// AZ-Delivery DHT11
#include "DHTesp.h"
#define DHT_GPIO 5   // signal GPIO5 (D1)

// Initialize DHT sensor
DHTesp dht;
float temperature;  // Read temperature as Celsius
float humidity;     // Read humidity in %

// AZ-Delivery Active Buzzer: GPIO16 (D0)
// This is used to leverage existing buitlin LED in nodemcu connected to GPIO16.
// int buzzer_gpio = 16;
// It is not required to configure it, as it is configured and managed later on the builtin led configuration

// GPIO0 (D3) is button FLASH on nodemcu PCB
int push_button_gpio = 0; // Flash button

// Status info
// First builtin LED, used to provide CO2 visual status info, as the buzzer produces sound info on the same pin on nodemcu v1.0 board
int co2_builtin_LED = 16;     // GPIO16 (D0), closer to the usb port, on the NodeMCU PCB
// Second builtin LED on  used to provide CO2 status info when sound alarm is deactivated
int status_builtin_LED = 2;   // GPIO2 (D4), on the ESP-12 module’s PCB
int alarm_ack = false;        // to indicate if push button has been pushed to ack the alarm and switch off the buzzer

// CO2 sensors
enum CO2_sensors {none, mhz14a, scd30}; // possible sensors integrated in the SW
CO2_sensors co2_sensor = none;

// CO2 device status
enum CO2_status {ok, warning, alarm}; // the device can have one of those CO2 status
CO2_status co2_device_status = ok;        // initialized to ok

// device status
boolean err_global = false;
boolean err_wifi = false;
boolean err_mqtt = false;
boolean err_co2 = false;
boolean err_dht = false;
boolean err_oled = false;

#include <Ticker.h>  //Ticker Library
Ticker blinker_co2_builtin_LED;    // to blink co2_builtin_LED and buzzer
Ticker blinker_status_builtin_LED;  // to blink status_builtin_LED

// flag to update OLED display from main loop instead of button ISR
boolean update_OLED_Status_flag = false;

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // Init config struct with default values
  eepromConfig.anaire_device_name = anaire_device_name;
  eepromConfig.CO2ppm_warning_threshold = CO2ppm_warning_threshold;
  eepromConfig.CO2ppm_alarm_threshold = CO2ppm_alarm_threshold;
  eepromConfig.cloud_server_address = cloud_server_address;
  eepromConfig.cloud_app_port = cloud_app_port;
  eepromConfig.local_alarm = true;
  eepromConfig.update_latest = false;

  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(115200);
  while (!Serial) {
    Serial.print("Attempting to open monitor serial port ");
    delay(3000); // wait 3 seconds for connection
  }

  // print info
  Serial.println();
  Serial.println();
  Serial.println("### INIT ANAIRE DEVICE ###########################################");

  // Initialize LEDs
  pinMode(co2_builtin_LED, OUTPUT);

  // Push button: attach interrupt to togle local report of CO2 status by blinking and buzzer
  pinMode(push_button_gpio, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(push_button_gpio), push_button_handler, FALLING);

  // Print welcome screen on OLED Display
  display.init();
  //display.flipScreenVertically();
  //display.clear();
  //display.setFont(ArialMT_Plain_24);
  //display.drawString(0, 0, "anaire.org");
  //display.display(); // update OLED display
  //delay(3000); // to show anaire.org

  // Wifi captive portal if double reset detection
  //WiFi.printDiag(Serial); //Remove this line if you do not want to see WiFi password printed
  if (WiFi.SSID() == "") {
    Serial.println("We haven't got any access point credentials, so get them now");
    initialConfig = true;
  } else {
    Serial.println("Access point credentials existing");
    initialConfig = false;
  }
  if (drd.detectDoubleReset()) {
    Serial.println("Double Reset Detected");
    initialConfig = true;
  } else {
    Serial.println("Double Reset NOT Detected");
    initialConfig = false;
  }

  // If a double reset was detected start configuration portal
  if (initialConfig) {

    Serial.println("Starting configuration portal...");

    // Update display to inform
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "CONFIG");
    display.drawString(0, 16, "ESP_" + String(anaire_device_id));
    display.display(); // update OLED display
  
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    wifiManager.setConfigPortalTimeout(600);

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal()) {
      Serial.println("Not connected to WiFi but continuing anyway.");
    } else {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
    }
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up
    // so resetting the device allows to go back into config mode again when it reboots.
    delay(5000);

    }

  else {

    // Put anaire.org in the display
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, "anaire.org");
    display.display(); // update OLED display
    delay(3000); // to show anaire.org
    
    // Read EEPROM config values
    Read_EEPROM();

    // Print config
    Print_Config();

    // Attempt to connect to WiFi network:
    Connect_WiFi();

    // Attempt to connect to MQTT broker
    if (!err_wifi) {
      Init_MQTT();
    }

    // Initialize and warm up device sensors
    Setup_sensors();

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  Serial.println ("--- LOOP BEGIN ---");

  // Timestamp for the loop start time
  control_loop_start = millis();

  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd.loop();

  // Read sensors
  Read_Sensors();

  // Evaluate CO2 value
  Evaluate_CO2_Value();

  // Update CO2 measurement on screen
  update_OLED_CO2();

  // Message the mqtt broker in the cloud app to send the measured values
  if (!err_wifi) {
    Send_Message_Cloud_App_MQTT();
  }

  // Complete time up to ControlLoopTimerDuration
  while ((millis() - control_loop_start) < control_loop_duration)
  {
    // Process wifi server requests
    Check_WiFi_Server();
    //delay(500);

    // if button was pressed flag to updated OLED was set
    if (update_OLED_Status_flag) {
      update_OLED_Status_flag = false;
      update_OLED_Status();
    }

    // Try to recover error conditions
    if (err_co2) {
      // Init co2 sensors
      Setup_sensors();
    }

    if (err_wifi) {
      // Attempt to connect to WiFi network:
      Connect_WiFi();
    }

    //Reconnect MQTT if needed
    if (!mqttClient.connected()) {
      err_mqtt = true;
    }

    //Reconnect MQTT if needed
    if ((err_mqtt) && (!err_wifi)) {
      // Attempt to connect to MQTT broker
      err_mqtt = false;
      mqttReconnect();
    }

    // if not there are not connectivity errors, receive mqtt messages
    if ((!err_mqtt) && (!err_wifi)) {
      mqttClient.loop();
    }

  }

  Serial.println("--- END LOOP");
  Serial.println();

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

// Connect to WiFi network
void Connect_WiFi() {

  Serial.print("Attempting to connect to Network named: ");
  Serial.println(wifi_ssid); // print the network name (SSID);

  // connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  //WiFi.begin(wifi_ssid, wifi_password);
  WiFi.begin();

  // Timestamp for connection timeout
  int wifi_timeout_start = millis();

  // Wait for warming time while blinking blue led
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifi_timeout_start) < WIFI_CONNECT_TIMEOUT)) {
    delay(500); // wait 0.5 seconds for connection
    Serial.print(".");
  }

  // Status
  if (WiFi.status() != WL_CONNECTED) {
    err_wifi = true;
    // Switch on blinking status_builtin_LED to reflect the error, with ALARM_BLINK_PERIOD
    //blinker_status_builtin_LED.attach_ms(ALARM_BLINK_PERIOD, changeState_status_builtin_LED);

  }
  else {
    err_wifi = false;
    // Switch off blinking status_builtin_LED
    //blinker_status_builtin_LED.detach();

    wifi_server.begin(); // start the web server on port 80
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Set mDNS to anaire_device_id.local
    if (!MDNS.begin(String(anaire_device_id))) {
      Serial.println("Error mDNS");
    }
    else {
      Serial.print("Hostname: ");
      Serial.print(String(anaire_device_id));
      Serial.println(".local");
    }

  }

}

void Init_MQTT() {

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.print(cloud_server_address);
  Serial.print(":");
  Serial.println(cloud_app_port);

  // Attempt to connect to MQTT broker
  mqttClient.setServer(cloud_server_address.c_str(), cloud_app_port);
  mqttClient.setCallback(Receive_Message_Cloud_App_MQTT);
  mqttClient.connect(anaire_device_id.c_str());

  if (!mqttClient.connected()) {
    err_mqtt = true;
    mqttReconnect();
  }
  else {
    err_mqtt = false;
    lastReconnectAttempt = 0;
    // Once connected resubscribe
    mqttClient.subscribe(mqtt_receive_topic.c_str());
    Serial.println("MQTT connected");
  }

}

// Print wifi status on serial monitor
void Print_WiFi_Status() {

  // Get current status
  //  WL_CONNECTED: assigned when connected to a WiFi network;
  //  WL_NO_SHIELD: assigned when no WiFi shield is present;
  //  WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
  //  WL_NO_SSID_AVAIL: assigned when no SSID are available;
  //  WL_SCAN_COMPLETED: assigned when the scan networks is completed;
  //  WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
  //  WL_CONNECTION_LOST: assigned when the connection is lost;
  //  WL_DISCONNECTED: assigned when disconnected from a network;

  //wifi_status = WiFi.status();
  Serial.print("wifi_status: ");
  Serial.println(WiFi.status());

  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  //ip_address = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Print the received signal strength:
  //wifi_rssi_dbm = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

}

void Check_WiFi_Server() {
  WiFiClient client = wifi_server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // Print current info
            client.print("Anaire Device ID: ");
            client.print(anaire_device_id);
            client.println("<br>");
            client.print("SW version: ");
            client.print(sw_version);
            client.println("<br>");
            client.print("CO2 PPM: ");
            client.print(CO2ppm_value);
            client.println("<br>");
            client.print("Temperature: ");
            client.print(temperature);
            client.println("<br>");
            client.print("Humidity: ");
            client.print(humidity);
            client.println("<br>");
            client.print("CO2ppm_alarm_threshold: ");
            client.print(CO2ppm_alarm_threshold);
            client.println("<br>");
            client.print("CO2ppm_warning_threshold: ");
            client.print(CO2ppm_warning_threshold);
            client.println("<br>");
            if (co2_sensor == scd30){
              client.print("CO2 Sensor: Sensirion SCD30");
              client.println("<br>");
              uint16_t val;
              SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO);
              //airSensor.getMeasurementInterval(&val);
              //client.print("SCD30 Measurement Interval: ");
              //client.print(val);
              //client.println("<br>");
              //airSensor.getAutoSelfCalibration();
              client.print("SCD30 AutoselfCalibration: ");
              client.print(airSensor.getAutoSelfCalibration());
              client.println("<br>");
              //airSensor.getForceRecalibration(&val);
              //client.print("SCD30 Force Recalibration: ");
              //client.print(val);
              //client.println("<br>");
              //airSensor.getTemperatureOffset(&val);
              //client.print("SCD30 Temperature Offset: ");
              //client.print(val);
              //client.println("<br>");
              //airSensor.getAltitudeCompensation(&val);
              //client.print("SCD30 AltitudeCompensation: ");
              //client.print(val);
              //client.println("<br>");
            }
            else {
              client.print("CO2 Sensor: Winsen MH-Z14A/Z19x");
              client.println("<br>");
              char myVersion[4];          
              myMHZ19.getVersion(myVersion);
              client.print("\nFirmware Version: ");
              for(byte i = 0; i < 4; i++)
              {
                client.print(myVersion[i]);
                if(i == 1)
                  client.print(".");    
              }
              client.println("<br>");
              client.print("Range: ");
              client.println(myMHZ19.getRange());   
              client.println("<br>");
              client.print("Background CO2: ");
              client.println(myMHZ19.getBackgroundCO2());
              client.println("<br>");
              client.print("Temperature Cal: ");
              client.println(myMHZ19.getTempAdjustment());
              client.println("<br>");
              client.print("ABC Status: "); myMHZ19.getABC() ? client.println("ON") :  client.println("OFF");
              client.println("<br>");
              
            }
            client.println("<br>");
            client.println("<br>");
            client.print("### CO2 STATUS: ");
            switch (co2_device_status) {
              case ok:
                client.print("OK");
                break;
              case warning:
                client.print("WARNING");
                break;
              case alarm:
                client.print("ALARM");
                break;
            }
            client.println("<br>");
            client.println("<br>");

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to calibrate the device.<br>");
            //client.print("Click <a href=\"/L\">here</a> turn the LED on pin 6 off<br>");

            client.println("<br>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {

          if (co2_sensor == mhz14a) {
            Calibrate_MHZ14A();
          }

          if (co2_sensor == scd30) {
            Calibrate_SCD30();
          }

        }
        if (currentLine.endsWith("GET /L")) {
          //digitalWrite(LED, LOW);                // GET /L turns the LED off
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }

}

// Detect and initialize co2 sensors
void Setup_sensors() {

  // Try Sensirion SCD-30 first

  // Init I2C bus for SCD30
  SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO);
  
  if (airSensor.begin(SCD30WIRE) == true) {

    Serial.println("Air sensor Sensirion SCD30 detected.");

    co2_sensor = scd30;

    // display device info
    SCD30DeviceInfo();

    // Set SCD30 (change in global variables at the start of SCD section in the initial section of this program)
    SCD30_Do_AutoSelfCalibration();
    SCD30_Do_Measurement_Interval();
    SCD30_Do_Temperature_Offset();
    SCD30_Do_Altitude_Compensation();
      
    // Timestamp for warming up start time
    int warming_up_start = millis();

    // Print info
    Serial.println ("Warming up SCD30 sensor...");

    // Wait for warming time while blinking blue led
    int counter = int(SCD30_WARMING_TIME / 1000);
    // Print welcome screen
    display.init();
    display.flipScreenVertically();
    while ((millis() - warming_up_start) < SCD30_WARMING_TIME) {
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "Anaire30ppm");
      display.drawString(0, 10, "ID " + String(anaire_device_id));
      display.drawString(0, 20, String(counter));
      display.display(); // update OLED display
      Serial.print(".");
      delay(500); // wait 500ms
      Serial.println(".");
      delay(500); // wait 500ms
      counter = counter - 1;
    }

    // Print info
    Serial.println ("Sensirion SCD30 CO2 sensor setup complete");

  }

  // Then MHZ14A
  else {

    // Initialize serial port to communicate with MHZ14A CO2 sensor. This is a software serial port
    //swSerial.begin(9600, SWSERIAL_8N1, swSerialRX_gpio, swSerialTX_gpio, false, 128);
    //Serial.println("swSerial Txd is on pin: " + String(swSerialTX_gpio));
    //Serial.println("swSerial Rxd is on pin: " + String(swSerialRX_gpio));

    mySerial.begin(MHZ_BAUDRATE);                           // Uno example: Begin Stream with MHZ19 baudrate  
    myMHZ19.begin(mySerial);                                // *Important, Pass your Stream reference here

    // Timestamp for serial up start time
    int serial_up_start = millis();

    while (((!mySerial) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
      Serial.println("Attempting to open serial port 1 to communicate to MHZ14A CO2 sensor");
      delay(1000); // wait 1 seconds for connection
    }

    // If the timeout was completed it is an error
    if ((millis() - serial_up_start) > MHZ14A_SERIAL_TIMEOUT) {
      err_co2 = true;
      co2_sensor = none;
      return;
    }

    else {

      Serial.println("CO2 sensor MH-Z14A/MH-Z19x detected.");

      // Disable ABC
      myMHZ19.autoCalibration(false);     // make sure auto calibration is off
      Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status
    
      char myVersion[4];          
      myMHZ19.getVersion(myVersion);

      Serial.print("\nFirmware Version: ");
      for(byte i = 0; i < 4; i++)
      {
        Serial.print(myVersion[i]);
        if(i == 1)
          Serial.print(".");    
      }

      Serial.print("Range: ");
      Serial.println(myMHZ19.getRange());   
      Serial.print("Background CO2: ");
      Serial.println(myMHZ19.getBackgroundCO2());
      Serial.print("Temperature Cal: ");
      Serial.println(myMHZ19.getTempAdjustment());
      Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");
   
      err_co2 = false;
      co2_sensor = mhz14a;

      // Timestamp for warming up start time
      int warming_up_start = millis();

      // Print info
      Serial.println ("Warming up MHZ14A CO2 sensor...");

      // Wait for warming time while blinking blue led
      int counter = int(MHZ14A_WARMING_TIME / 1000);
      // Print welcome screen
      display.init();
      display.flipScreenVertically();
      while ((millis() - warming_up_start) < MHZ14A_WARMING_TIME) {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, "Anaire50ppm");
        display.drawString(0, 10, "ID " + String(anaire_device_id));
        display.drawString(0, 20, String(counter));
        display.display(); // update OLED display
        Serial.print(".");
        delay(500); // wait 500ms
        Serial.println(".");
        delay(500); // wait 500ms
        counter = counter - 1;
      }

      // Print info
      Serial.println ("Warming up MHZ14A CO2 sensor complete");

      // Print info
      Serial.println ("MHZ14A CO2 sensor setup complete");

      // Setup DHT11
      dht.setup(DHT_GPIO, DHTesp::DHT11);

    }

  }

}

// Read sensors
void Read_Sensors() {

  switch (co2_sensor) {
    case scd30:
      Read_SCD30(); // Read co2, temperature and humidity
      break;
    case mhz14a:
      Read_MHZ14A(); // Read co2
      Read_DHT11();  // Read temperature and humidity
      break;
    case none:
      break;
  }
}

// Read MHZ14A CO2 sensor
void Read_MHZ14A() {

  // Timestamp for serial up start time
  int serial_up_start = millis();

  // clears out any garbage in the RX buffer
  while (((mySerial.available()) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
    int garbage = mySerial.read();
    delay(100);
    Serial.println ("Cleaning mySerial data...");
  }

  // Send out read command to the sensor - 9 bytes
  mySerial.write(measurement_command, 9);

  // pauses the sketch and waits for the TX buffer to send all its data to the sensor
  mySerial.flush();

  // Timestamp for serial up start time
  serial_up_start = millis();

  // pauses the sketch and waits for the sensor response
  if (((!mySerial.available()) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
    Serial.println ("Waiting for mySerial data...");
    delay(1000);
  }

  // once data is available, it reads it to a variable 9 bytes
  mySerial.readBytes(response_CO2, 9);

  // calculates CO2ppm value
  response_CO2_high = (int)response_CO2[2];
  response_CO2_low = (int)response_CO2[3];
  CO2ppm_value = (256 * response_CO2_high) + response_CO2_low;

  // prints calculated CO2ppm value to serial monitor
  Serial.print ("MHZ14A CO2ppm_value: ");
  Serial.println (CO2ppm_value);

}

// Calibrate MHZ14A sensor
// Requires that the device is localed on open air for several minutes
void Calibrate_MHZ14A() {

  // Print info
  Serial.println ("Calibrating Winsen MHZ14A/MHZ19x CO2 sensor...");

  // Write calibration command
  //mySerial.write(calibration_command, 9);

  // Waits for 3 seconds for the command to take effect
  delay(3000);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  // Wait for calibrating time to stabalise...
  int counter = MHZ14A_CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < MHZ14A_CALIBRATION_TIME) {
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Calibrando");
    display.drawString(0, 16, "MHZ14A " + String(counter));
    display.display(); // update OLED display
    Serial.print(".");
    delay(500);
    Serial.println(".");
    delay(500);
    counter = counter - 1;
  }

  // Take a reading which be used as the zero point for 400 ppm 
  myMHZ19.calibrate();    
  
  // Print info
  Serial.println ("MHZ14A CO2 sensor calibrated");

}

// Read Sensirion SCD30 CO2, humidity and temperature sensor
void Read_SCD30()
{

  SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO);
  
  if (airSensor.dataAvailable())
  {
    CO2ppm_value = airSensor.getCO2();
    temperature = airSensor.getTemperature();
    humidity = airSensor.getHumidity();

    Serial.print("SCD30 co2(ppm): ");
    Serial.print(CO2ppm_value);

    Serial.print(" temp(C): ");
    Serial.print(temperature, 1);

    Serial.print(" humidity(%): ");
    Serial.print(humidity, 1);

    Serial.println();
  }
  else {
    Serial.println("SCD30 No data available");
  }

}

// Calibrate Sensirion SCD30 CO2, humidity and temperature sensor
// Requires that the device is localed on open air for several minutes
void Calibrate_SCD30() {

  // Print info
  Serial.println ();
  Serial.println ("Calibrating SCD30 sensor...");
  
  // Timestamp for calibrating start time
  int calibrating_start = millis();

  // Set 2 seconds between measurements, required at least 2 minutes prior to calibration
  //airSensor.setMeasurementInterval(2);
  
  // Wait for calibrating time while reading values at maximum speed
  int counter = SCD30_CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < SCD30_CALIBRATION_TIME) {
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Calibrando");
    display.drawString(0, 16, "SCD30 " + String(counter));
    display.display(); // update OLED display
    Serial.print(".");
    delay(500);
    Serial.println(".");
    delay(500);
    counter = counter - 1;
  }
  
  // Send forced calibration command setting as zero reference value
  SCD30_Do_Forced_Calibration_Factor();

  delay(2000);
  
  // Restore Measurement Interval
  //SCD30_Do_Measurement_interval();

  // Print info
  Serial.println ("SCD30 sensor calibrated");

}

void SCD30_Do_AutoSelfCalibration()
{

  //airSensor.setAutoSelfCalibration(SCD30_AutoSelfCalibration);
  Serial.print("Reading SCD30 AutoSelfCalibration: ");
  Serial.println(airSensor.getAutoSelfCalibration());

  /*
  //Commented until getAutoSelfCalibration is implemented
  uint16_t val;
  //if (airSensor.getAutoSelfCalibration(&val)) {
  if (airSensor.getAutoSelfCalibration()) {
  
    Serial.print("\nReading SCD30 AutoSelfCalibration before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 AutoSelfCalibration to: ");
    Serial.println(SCD30_AutoSelfCalibration);

    if (airSensor.setAutoSelfCalibration(SCD30_AutoSelfCalibration)) {

        if (airSensor.getAutoSelfCalibration(&val)) {
          Serial.print("Reading SCD30 AutoSelfCalibration after change: ");
          Serial.println(val);
        }
        else {
          Serial.println("Could not obtain SCD30 AutoSelfCalibration");
        }
    }
    else {
      Serial.println("Could not set new SCD30 AutoSelfCalibration");
    }
  }
  else {
    Serial.println("Could not obtain AutoSelfCalibration");
  }
  */
}

void SCD30_Do_Temperature_Offset()
{
  uint16_t val;

  airSensor.setTemperatureOffset(SCD30_TEMPERATURE_OFFSET);
  
  /*
  if (airSensor.getTemperatureOffset(&val)) {
    
    Serial.print("\nReading SCD30 Temperature Offset before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Temperature Offset to: ");
    Serial.println(SCD30_TEMPERATURE_OFFSET);

    if ( airSensor.setTemperatureOffset(SCD30_TEMPERATURE_OFFSET) ) {

        if ( airSensor.getTemperatureOffset(&val) ) {
          Serial.print("Reading SCD30 Temperature Offset after making change: ");
          Serial.println(val);
        }
        else {
          Serial.println("Could not obtain SCD30 Temperature Offset");
        }
    }
    else {
      Serial.println("Could not set new SCD30 Temperature Offset");
    }
  }
  else {
    Serial.println("Could not obtain Temperature Offset");
  }

  */
  
}

void SCD30_Do_Measurement_Interval()
{
  uint16_t val;

//  if ( airSensor.getMeasurementInterval(&val) ) {
//    Serial.print("\nReading SCD30 Measurement Interval before change: ");
//    Serial.println(val);

    Serial.print("Setting SCD30 new Measurement Interval to: ");
    Serial.println(SCD30_MEASUREMENT_INTERVAL);

    if ( airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL) ) {
      Serial.print("Reset SCD30 Measurement Interval to: ");
      Serial.println(SCD30_MEASUREMENT_INTERVAL);

      /*
      if ( airSensor.getMeasurementInterval(&val) ) {
        Serial.print("Reading SCD30 Measurement Interval after change: ");
        Serial.println(val);       
      }
      else {
        Serial.println("Could not obtain SCD30 Measurement Interval");
      }
      */
    }
    else {
      Serial.print("Could not reset SCD30 Measurement Interval to ");
      Serial.println(SCD30_MEASUREMENT_INTERVAL);
    }    
    
//  }
//  else {
//    Serial.println("Could not obtain SCD30 Measurement Interval");
//  }
}

void SCD30_Do_Forced_Calibration_Factor()
{
  uint16_t val;

//  airSensor.setForceRecalibration(SCD30_FORCED_CALIBRATION);
  
/*
  if ( airSensor.getForceRecalibration(&val) ) {
    Serial.print("\nReading SCD30 Forced Calibration Factor before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Forced Calibration Factor to: ");
    Serial.println(SCD30_FORCED_CALIBRATION);

    if (airSensor.setForceRecalibration(SCD30_FORCED_CALIBRATION)) {

        
        if (airSensor.getForceRecalibration(&val)) {

          Serial.print("Reading SCD30 Forced Calibration Factor after change: ");
          Serial.println(val);
        }
        else {
          Serial.println("Could not obtain SCD30 forced calibration factor");
        }
    }
    else {
      Serial.println("Could not set SCD30 Forced Calibration Factor");
    }
  }
  else {
    Serial.println("Could not obtain SCD30 Forced Calibration Factor");
  }

*/
}

void SCD30_Do_Altitude_Compensation()
{
  uint16_t val;

  /* paulvha : you can set EITHER the Altitude compensation of the pressure.
   * Setting both does not make sense as both overrule each other, but it is included for demonstration
   *
   * see Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
   *
   *    The CO2 measurement value can be compensated for ambient pressure by feeding the pressure value in mBar to the sensor.
   *    Setting the ambient pressure will overwrite previous and future settings of altitude compensation. Setting the argument to zero
   *    will deactivate the ambient pressure compensation. For setting a new ambient pressure when continuous measurement is running
   *    the whole command has to be written to SCD30.
   *
   *    Setting altitude is disregarded when an ambient pressure is given to the sensor
   */ 

  airSensor.setAltitudeCompensation(SCD30_ALTITUDE_COMPENSATION);
  /*
  if ( airSensor.getAltitudeCompensation(&val) ) {
    Serial.print("\nReading SCD30 Altitude Compensation before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Altitude Compensation to: ");
    Serial.println(SCD30_ALTITUDE_COMPENSATION);

    if (airSensor.setAltitudeCompensation(SCD30_ALTITUDE_COMPENSATION)) {

        if (airSensor.getAltitudeCompensation(&val)) {
          Serial.print("Reading SCD30 Altitude Compensation after change: ");
          Serial.println(val);
        }
        else {
          Serial.println("Could not obtain SCD30 Altitude Compensation");
        }
    }
    else {
      Serial.println("Could not set new SCD30 Altitude Compensation");
    }
  }
  else {
    Serial.println("Could not obtain SCD30 Altitude Compensation");
  }
  */
}

void SCD30DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) +1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 33 digits (32 serial + 0x0)

  /*
  if (airSensor.getSerialNumber(buf))
  {
   Serial.print(F("SCD30 serial number: "));
   Serial.println(buf);
  }

  // read Firmware level
  if ( airSensor.getFirmwareLevel(val) ) {
    Serial.print("SCD30 Firmware level: Major: ");
    Serial.print(val[0]);

    Serial.print("\t, Minor: ");
    Serial.println(val[1]);
  }
  else {
    Serial.println("Could not obtain firmware level");
  }
  */
}

// Evaluate CO2 value versus warning and alarm threasholds and process CO2 alarm information
void Evaluate_CO2_Value() {

  // Status: ok

  // Recovering to "ok" status stops any warning or alarm warnings and quits special mode (after pressing flash button), therefore after passing by "ok" status
  // the device is "reset" and when entering warning or alarm state the device will report localy again by blinking o2_builtin_led_gpio16 led and the buzzer,
  // and pushing the flash button will be required if the user wanrts to stop lignt and sound alerting

  if (CO2ppm_value < CO2ppm_warning_threshold) {
    co2_device_status = ok; // update co2 status
    blinker_co2_builtin_LED.detach(); // stop co2_builtin_LED and buzzer blinking
    digitalWrite(co2_builtin_LED, LOW); // update co2_builtin_LED_gpio1 (always on) buzzer (off) status
    blinker_status_builtin_LED.detach(); // stop blinkg status_builtin_LED_gpio16
    alarm_ack = false; // Init alarm ack status
  }

  // Status: warning
  else if ((CO2ppm_value >= CO2ppm_warning_threshold) && (CO2ppm_value < CO2ppm_alarm_threshold)) {
    co2_device_status = warning; // update device status
    if (!alarm_ack) { // flash button hasn't been pressed
      blinker_co2_builtin_LED.attach_ms(WARNING_BLINK_PERIOD, changeState_co2_builtin_LED); // warning blink of light and sound
      blinker_status_builtin_LED.attach_ms(WARNING_BLINK_PERIOD, changeState_status_builtin_LED); // warning blink of light on status LED
    }
  }

  // Status: alarm
  else {
    co2_device_status = alarm; // update device status
    if (!alarm_ack) { // flash button hasn't been pressed
      blinker_co2_builtin_LED.attach_ms(ALARM_BLINK_PERIOD, changeState_co2_builtin_LED); // warning blink of light and sound
      blinker_status_builtin_LED.attach_ms(ALARM_BLINK_PERIOD, changeState_status_builtin_LED); // warning blink of light on status LED
    }
  }

  // Print info on serial monitor
  switch (co2_device_status) {
    case ok:
      Serial.println ("STATUS: OK");
      break;
    case warning:
      Serial.println ("STATUS: WARNING");
      break;
    case alarm:
      Serial.println ("STATUS: ALARM");
      break;
  }

}

// Read temperature and humidity values from DHT11
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
void Read_DHT11() {

  TempAndHumidity lastValues = dht.getTempAndHumidity();

  // Read humidity as percentage
  humidity = lastValues.humidity;

  // Read temperature as Celsius
  temperature = lastValues.temperature;

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    err_dht = true;
    humidity = 0;
    temperature = 0;
  }
  else {
    err_dht = false;
    Serial.print("DHT11 Humidity: ");
    Serial.print(humidity);
    Serial.print(" % \n");
    Serial.print("DHT11 Temperature: ");
    Serial.print(temperature);
    Serial.println("ºC");
  }

}

// Send measurements to the cloud application by MQTT
void Send_Message_Cloud_App_MQTT() {

  // Print info
  Serial.print("Sending mqtt message to the send topic ");
  Serial.println(mqtt_send_topic);
  sprintf(mqtt_message, "{id: %s,CO2: %d,humidity: %f,temperature: %f}", anaire_device_id.c_str(), CO2ppm_value, humidity, temperature);
  Serial.print(mqtt_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents
  mqttClient.publish(mqtt_send_topic.c_str(), mqtt_message);

}

// callback function to receive configuration messages from the cloud application by MQTT
void Receive_Message_Cloud_App_MQTT(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> jsonBuffer;
  char received_payload[128];
  memcpy(received_payload, payload, length);
  Serial.print("Message arrived: ");
  Serial.println(received_payload);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(jsonBuffer, received_payload);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  //JsonObject obj = jsonBuffer.as<JsonObject>();
  //Serial.println(jsonBuffer["warning"]);
  //Serial.println(jsonBuffer["caution"]);
  //Serial.println(jsonBuffer["alarm"]);
  //Serial.println(jsonBuffer["name"]);
  //Serial.println(jsonBuffer["update"]);
  //Serial.println(jsonBuffer);

  // fill the values on the eeprom config struct
  //eepromConfig.anaire_device_name = String(jsonBuffer["name"]);
  eepromConfig.CO2ppm_warning_threshold = (int)jsonBuffer["warning"];
  eepromConfig.CO2ppm_alarm_threshold = (int)jsonBuffer["caution"];

  // update global variables
  //anaire_device_name = eepromConfig.anaire_device_name;
  CO2ppm_warning_threshold = eepromConfig.CO2ppm_warning_threshold;
  CO2ppm_alarm_threshold = eepromConfig.CO2ppm_alarm_threshold;

  // update status over new thresholds
  Evaluate_CO2_Value();

  // Use update flag to wipe EEPROM and update to latest bin
  if (jsonBuffer["update"] == "ON") {

    boolean result = EEPROM.wipe();
    if (result) {
      Serial.println("All EEPROM data wiped");
    } else {
      Serial.println("EEPROM data could not be wiped from flash store");
    }

    // update firmware to latest bin
    firmware_update();

  }

  //print info
  Serial.println("MQTT update ->");
  Print_Config();

  // save the new values
  Write_EEPROM();

}

// MQTT reconnect function
void mqttReconnect() {
  //Try to reconnect only if it has been more than 5 sec since last attemp
  unsigned long now = millis();
  if (now - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = now;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(anaire_device_id.c_str())) {
      Serial.println("MQTT connected");
      lastReconnectAttempt = 0;
      err_mqtt = false;
      // Once connected resubscribe
      mqttClient.subscribe(mqtt_receive_topic.c_str());
    } else {
      err_mqtt = true;
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

// ISR to respond to button pressing to toggle local alarm reporting thoriugh builtin LED and buzzer
ICACHE_RAM_ATTR void push_button_handler() {

  if (!alarm_ack) {

    // Print info
    Serial.println ("FLASH Push button interrupt - alarm_ack ON");

    // Switch off the buzzer and co2_builtin_LED, as in normal status
    blinker_co2_builtin_LED.detach(); //Use attach_ms if you need time in ms 
    digitalWrite(co2_builtin_LED, LOW);

    // Switch on blinking status_builtin_LED to reflect special mode, using WARNING_BLINK_PERIOD
    //blinker_status_builtin_LED.attach_ms(WARNING_BLINK_PERIOD, changeState_status_builtin_LED);

    alarm_ack = true; // alarm has been ack

  }

  else {

    // Print info
    Serial.println ("FLASH Push button interrupt - alarm_ack OFF");

    // Switch off blinking status_builtin_LED to reflect normal mode
    //blinker_status_builtin_LED.detach();

    alarm_ack = false; // alarm has been reset

    // Evaluate last CO2 measurement and update led and buzzer accordingly
    Evaluate_CO2_Value();

  }

  // Print id and IP info in OLED display
  update_OLED_Status_flag = true;

}

// To blink on co2_builtin_LED and buzzer
void changeState_co2_builtin_LED() {
  digitalWrite(co2_builtin_LED, !(digitalRead(co2_builtin_LED)));  //Invert Current State of LED co2_builtin_LED
}

// To blink on status_builtin_LED
void changeState_status_builtin_LED() {
  digitalWrite(status_builtin_LED, !(digitalRead(status_builtin_LED)));  //Invert Current State of LED status_builtin_LED
}

// Update CO2 info on OLED display
void update_OLED_CO2() {

  // setup display and text format
  display.init();
  display.flipScreenVertically();
  display.clear();
  
  // display CO2 measurement
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(4, 4, String(CO2ppm_value) + "ppm");
  
  /*
  // display CO2 measurement on first line
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, String(CO2ppm_value) + "ppm " + String(int(temperature)) + "º " + String(int(humidity)) + "%");

  // if there is an error display it on third line
  if (err_wifi) {
    display.drawString(0, 16, "err wifi");
  }
  else if (err_mqtt) {
    display.drawString(0, 16, "err mqtt");
  }
  else if (err_co2) {
    display.drawString(0, 16, "err co2");
  }
  else if (err_dht) {
    display.drawString(0, 16, "err dht");
  }
  else { // status
    switch (co2_device_status) {
      case ok:
        display.drawString(0, 16, "CO2 BIEN");
        break;
      case warning:
        display.drawString(0, 16, "CO2 REGULAR");
        break;
      case alarm:
        display.drawString(0, 16, "¡¡¡ CO2 MAL !!!");
        break;
    }
  }
  */
  
  display.display(); // update OLED display

}

// Update device status on OLED display
void update_OLED_Status() {

  // setup display and text format
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // display device id on first line
  display.drawString(0, 0, "ID " + String(anaire_device_id));

  // display IP address on second line
  String ipaddress = WiFi.localIP().toString();
  display.drawStringMaxWidth(0, 10, 128, ipaddress);

  // if there is an error display it on third line
  if (err_dht) {
    display.drawString(0, 20, "err dht");
  }
  if (err_mqtt) {
    display.drawString(0, 20, "err mqtt");
  }
  if (err_wifi) {
    display.drawString(0, 20, "err wifi");
  }
  if (err_co2) {
    display.drawString(0, 20, "err co2");
  }
  
  if ((!err_dht) && (!err_mqtt) && (!err_wifi) && (!err_co2))  {
    //display.drawString(0, 20, "device ok");
    display.drawString(0, 20, sw_version);
  }

  display.display(); // update OLED display
}

void Read_EEPROM () {

  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyEEPROMStruct));

  /*
    // Wipe EEPROM
    boolean result = EEPROM.wipe();
    if (result) {
    Serial.println("All EEPROM data wiped");
    } else {
    Serial.println("EEPROM data could not be wiped from flash store");
    }
  */

  // Check if the EEPROM contains valid data from another run
  // If so, overwrite the 'default' values set up in our struct
  if (EEPROM.percentUsed() >= 0) {

    Serial.println("EEPROM has data from a previous run.");
    Serial.print(EEPROM.percentUsed());
    Serial.println("% of ESP flash space currently used");

    // Read saved data
    EEPROM.get(0, eepromConfig);

    // Restore config values
    anaire_device_name = eepromConfig.anaire_device_name;
    CO2ppm_warning_threshold = eepromConfig.CO2ppm_warning_threshold;
    CO2ppm_alarm_threshold = eepromConfig.CO2ppm_alarm_threshold;
    //wifi_ssid = eepromConfig.wifi_ssid;
    //wifi_password = eepromConfig.wifi_password;
    cloud_server_address = eepromConfig.cloud_server_address;
    cloud_app_port = eepromConfig.cloud_app_port;

    Print_Config();

  } else {
    Serial.println("No EEPROM data - using default config values");
  }
}

void Write_EEPROM () {

  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyEEPROMStruct));

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromConfig);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "EEPROM Commit OK" : "EEPROM Commit failed");

}

void Print_Config() {
  Serial.println("#######################################");
  Serial.print("Device ID: ");
  Serial.println(anaire_device_id);
  Serial.print("SW: ");
  Serial.println(sw_version);
  Serial.print("anaire_device_name: ");
  Serial.println(anaire_device_name);
  Serial.print("CO2ppm_warning_threshold: ");
  Serial.println(CO2ppm_warning_threshold);
  Serial.print("CO2ppm_alarm_threshold: ");
  Serial.println(CO2ppm_alarm_threshold);
  Serial.print("wifi_ssid: ");
  Serial.println(wifi_ssid);
  Serial.print("cloud_server_address: ");
  Serial.println(cloud_server_address);
  Serial.print("cloud_app_port: ");
  Serial.println(cloud_app_port);
  Serial.print("local_alarm: ");
  Serial.println(eepromConfig.local_alarm);
  Serial.print("update_latest: ");
  Serial.println(eepromConfig.update_latest);
  Serial.println("#######################################");
}

void firmware_update() {

  // Add optional callback notifiers
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);

  BearSSL::WiFiClientSecure UpdateClient;
  UpdateClient.setInsecure();

  // Run http update
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin");

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }

}

void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}
