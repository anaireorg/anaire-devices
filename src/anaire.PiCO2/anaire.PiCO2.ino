//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Anaire PiCO2 - Open CO2, temperature and humidity measurement device
// www.anaire.org
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parts
//   TTGO T-Display board, Sensirion SCD30 CO2 temp and humidity sensor, AZ delivery active buzzer, 3D Box designed by Anaire, 3.7V lithium battery JST Connector 2Pin 1.25mm
//
// SW Setup:
//   Setup the Arduino IDE for the ESP32 platform: https://github.com/espressif/arduino-esp32
//   Setup the required libraries:
//   - TTGO T-Display's Git: Download the code as zip https://github.com/Xinyuan-LilyGO/TTGO-T-Display/archive/master.zip
//     Extract it and copy the Folder TFT_eSPI to your Arduino library path (usually <your user>/Documents/Arduino in Windows)
//   - The Sensirion Gadget BLE Arduino Library https://github.com/Sensirion/Sensirion_GadgetBle_Arduino_Library/releases
//     Download latest zip. In the Arduino IDE, select Sketch -> include Library -> Add .zip Library and select the downloaded .zip file
//   - Install the following libraries from Arduino IDE, select Tools -> Library manager.
//       - Search for Adafruit SCD30 and install the library
//       - Search for WifiManager by tzapu,tablatronix https://github.com/tzapu/WiFiManager and install the library
//
// Buttons design:
//   Top button click: toggles buzzer sound; enabled by default
//   Top button double click: performs SCD30 forced calibration
//   Top button triple click: enables self calibration
//   Bottom button click: sleep; click again the button to wake up
//   Bottom button double click: restart device
//   Bottom button triple click: starts captive portal
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String sw_version = "v3.20210418.PiCO2";
// 202104xx First fully functional Anaire device on TTGo T-Display board, connected to Anaire Cloud App
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen, in HEX format. Example: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String anaire_device_id;

// Init to default values; if they have been chaged they will be readed later, on initialization
struct MyConfigStruct {
  char anaire_device_name[24] = "";                           // Device name; default to anaire_device_id
  uint16_t CO2ppm_warning_threshold = 700;                    // Warning threshold; default to 700ppm
  uint16_t CO2ppm_alarm_threshold = 1000;                     // Alarm threshold; default to 1000ppm
  char MQTT_server[24] = "mqtt.anaire.org";                   // MQTT server url or public IP address. Default to Anaire Portal on portal.anaire.org
  uint16_t MQTT_port = 80;                                    // MQTT port; Default to Anaire Port on 30183
  boolean sound_alarm = true;                                 // Global flag to control sound alarm; default to true
  boolean ABC = false;                                        // Automatic Baseline Correction of CO2 sensor; default to false
  uint16_t FRC_value = 420;                                   // Forced ReCalibration value; default to 420ppm
  uint16_t temperature_offset = 0;                            // temperature offset for SCD30 CO2 measurements
  uint16_t altitude_compensation = 0;                         // altitude compensation for SCD30 CO2 measurements
  char wifi_user[24] = "";                                    // WiFi user to be used on WPA Enterprise. Default to null (not used)
  char wifi_password[24] = "";                                // WiFi password to be used on WPA Enterprise. Default to null (not used)
  //char wifi_user[24] = "HI\\alopezg";                                    // WiFi user to be used on WPA Enterprise. Default to null (not used)
  //char wifi_password[24] = "0Inloca6";                                // WiFi password to be used on WPA Enterprise. Default to null (not used)
} eepromConfig;

// to store data on nvs partition
#include <Preferences.h>
Preferences preferences;

// Measurements
int CO2ppm_value = 0;         // CO2 ppm measured value
int CO2ppm_accumulated = 0;   // Accumulates co2 measurements for a MQTT period
int CO2ppm_samples = 0;       // Counts de number of samples for a MQTT period
float temperature;            // Read temperature as Celsius
float humidity;               // Read humidity in %

// CO2 sensors
enum CO2_sensors {none, scd30_sensor}; // possible sensors integrated in the SW
CO2_sensors co2_sensor = none;

// CO2 device status
enum co2_status {co2_ok, co2_warning, co2_alarm}; // the device can have one of those CO2 status
co2_status co2_device_status = co2_ok; // initialized to ok

// device status
boolean err_global = false;
boolean err_wifi = false;
boolean err_MQTT = false;
boolean err_sensor = false;

// Measurements loop: time between measurements
unsigned int measurements_loop_duration = 5000;   // 5 seconds
unsigned long measurements_loop_start;            // holds a timestamp for each control loop start

// MQTT loop: time between MQTT measurements sent to the cloud
unsigned int MQTT_loop_duration = 30000;          // 30 seconds
unsigned long MQTT_loop_start;                    // holds a timestamp for each cloud loop start
unsigned long lastReconnectAttempt = 0;           // MQTT reconnections

// Errors loop: time between error condition recovery
unsigned int errors_loop_duration = 3000;         // 3 seconds
unsigned long errors_loop_start;                  // holds a timestamp for each error loop start

// TTGO ESP32 board
#include "esp_timer.h"
#include <Wire.h>

// Display and fonts
#include <TFT_eSPI.h>
#include <SPI.h>
#include "SensirionSimple25pt7b.h"
#include "ArchivoNarrow_Regular10pt7b.h"
#include "ArchivoNarrow_Regular50pt7b.h"
#define GFXFF 1
#define FF99  &SensirionSimple25pt7b
#define FF90  &ArchivoNarrow_Regular10pt7b
#define FF95  &ArchivoNarrow_Regular50pt7b
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke library, pins defined in User_Setup.h

// Customized Anaire splash screen
#include "anaire_ttgo_splash.h"

// Buttons: Top and bottom considered when USB connector is positioned on the right of the board
#include "Button2.h"
#define BUTTON_TOP 35
#define BUTTON_BOTTOM  0
Button2 button_top(BUTTON_TOP);
Button2 button_bottom(BUTTON_BOTTOM);

// Sensirion SCD30 CO2, temperature and humidity sensor
#include <Adafruit_SCD30.h>
Adafruit_SCD30 scd30;
#define SCD30_SDA_pin 26  // Define the SDA pin used for the SCD30
#define SCD30_SCL_pin 27  // Define the SCL pin used for the SCD30

// Bluetooth in TTGO T-Display
#include "Sensirion_GadgetBle_Lib.h"  // to connect to Sensirion MyAmbience Android App available on Google Play
GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_CO2_ALT);

// AZ-Delivery Active Buzzer
#define BUZZER_GPIO 12 // signal GPIO12 (pin TOUCH5/ADC15/GPIO12 on TTGO)
bool sound = true;

// control loop timing
static int64_t lastMmntTime = 0;
static int startCheckingAfterUs = 5000000; // 5s

// to indicate if push button has been pushed to ack the alarm and switch off the buzzer
boolean alarm_ack = false;

// WiFi
#include "WiFi.h"
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
const int WIFI_CONNECT_TIMEOUT = 10000;           // 10 seconds
int wifi_status = WL_IDLE_STATUS;
WiFiServer wifi_server(80);                       // to check if it is alive
WiFiClient wifi_client;
#define EAP_IDENTITY eepromConfig.wifi_user       // if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_PASSWORD eepromConfig.wifi_password   // your Eduroam password
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

/*
  #if defined(ESP32)
  #define USE_SPIFFS            true
  #define ESP_DRD_USE_EEPROM    true
  #else
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
  #endif
*/

// MQTT
#include <PubSubClient.h>
char MQTT_message[256];
PubSubClient MQTT_client(wifi_client);
char received_payload[384];
String MQTT_send_topic;
String MQTT_receive_topic;
  
//JSON
#include <ArduinoJson.h>
StaticJsonDocument<384> jsonBuffer;


///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////
void Connect_WiFi() {

  Serial.println("Attempting to connect to WiFi network");

  // Set wifi mode
  WiFi.mode(WIFI_STA);

  // If there are not wifi user and wifi password defined, proceed to traight forward configuration
  if ((strlen(eepromConfig.wifi_user) == 0) && (strlen(eepromConfig.wifi_password) == 0)) {
    Serial.println("Attempting to authenticate with WPA2");
    WiFi.begin();
  }
  else {  // set up wpa2 enterprise
    Serial.println("Attempting to authenticate with WPA2 Enterprise ");
    Serial.print("User: ");
    Serial.println(EAP_IDENTITY);
    Serial.print("Password: ");
    Serial.println(EAP_PASSWORD);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //provide username --> identity and username is same
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD)); //provide password
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
    esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function
    //WiFi.begin("IntranetTelefonicaWiFi"); //connect to wifi
    WiFi.begin(); //connect to wifi
  }

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
    Serial.println(" WiFi not connected");
  }
  else {
    err_wifi = false;
    Serial.println(" WiFi connected");
    // start the web server on port 80
    wifi_server.begin();
  }
  Print_WiFi_Status();
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

  Serial.print("wifi_status: ");
  Serial.println(WiFi.status());

  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  //ip_address = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Print your WiFi shield's MAC address:
  Serial.print("MAC Adress: ");
  Serial.println(WiFi.macAddress());

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
            client.print("ANAIRE PiCO2 DEVICE");
            client.println("<br>");
            client.print("SW version: ");
            client.print(sw_version);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("Anaire Device ID: ");
            client.print(anaire_device_id);
            client.println("<br>");
            client.print("Anaire Device name: ");
            client.print(eepromConfig.anaire_device_name);
            client.println("<br>");
            client.print("IP Adress: ");
            client.print(WiFi.localIP());
            client.println("<br>");
            client.print("MAC Adress: ");
            client.print(WiFi.macAddress());
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("CO2ppm_alarm_threshold: ");
            client.print(eepromConfig.CO2ppm_alarm_threshold);
            client.println("<br>");
            client.print("CO2ppm_warning_threshold: ");
            client.print(eepromConfig.CO2ppm_warning_threshold);
            client.println("<br>");
            client.print("MQTT Server: ");
            client.print(eepromConfig.MQTT_server);
            client.println("<br>");
            client.print("MQTT Port: ");
            client.print(eepromConfig.MQTT_port);
            client.println("<br>");
            client.print("Sound alarm: ");
            client.print(eepromConfig.sound_alarm);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            if (co2_sensor == scd30_sensor) {
              client.print("CO2 Sensor: Sensirion SCD30");
              client.println("<br>");
              uint16_t val;
              //scd30.getMeasurementInterval(&val);
              client.print("Measurement Interval: ");
              client.print(val);
              client.println("<br>");
              client.print("AutoselfCalibration: ");
              //client.print(scd30.getAutoSelfCalibration());
              client.println("<br>");
              //scd30.getForceRecalibration(&val);
              client.print("Force Recalibration: ");
              client.print(val);
              client.println("<br>");
              //scd30.getTemperatureOffset(&val);
              client.print("Temperature Offset: ");
              client.print(val);
              client.println("<br>");
              //scd30.getAltitudeCompensation(&val);
              client.print("AltitudeCompensation: ");
              client.print(val);
              client.println("<br>");
            }
            client.println("------");
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
            client.print("CO2 STATUS: ");
            switch (co2_device_status) {
              case co2_ok:
                client.print("OK");
                break;
              case co2_warning:
                client.print("WARNING");
                break;
              case co2_alarm:
                client.print("ALARM");
                break;
            }
            client.println("<br>");
            client.println("------");
            client.println("<br>");

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to calibrate the device.<br>");
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

        // Check to see if the client request was "GET /H" to calibrate the sensor:
        if (currentLine.endsWith("GET /H")) {
          if (co2_sensor == scd30_sensor) {
            //Calibrate_SCD30();
          }
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }

}
void Init_MQTT() {
  Serial.print("Attempting to connect to the MQTT broker ");
  Serial.print(eepromConfig.MQTT_server);
  Serial.print(":");
  Serial.println(eepromConfig.MQTT_port);

  // Attempt to connect to MQTT broker
  //MQTT_client.setBufferSize(512); // to receive messages up to 512 bytes length (default is 256)
  MQTT_client.setServer(eepromConfig.MQTT_server, eepromConfig.MQTT_port);
  MQTT_client.setCallback(Receive_Message_Cloud_App_MQTT);
  MQTT_client.connect(anaire_device_id.c_str());

  if (!MQTT_client.connected()) {
    err_MQTT = true;
    MQTTReconnect();
  }
  else {
    err_MQTT = false;
    lastReconnectAttempt = 0;
    // Once connected resubscribe
    MQTT_client.subscribe(MQTT_receive_topic.c_str());
    Serial.print("MQTT connected - Receive topic: ");
    Serial.println(MQTT_receive_topic);
  }

}
// MQTT reconnect function
void MQTTReconnect() {
  //Try to reconnect only if it has been more than 5 sec since last attemp
  unsigned long now = millis();
  if (now - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = now;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTT_client.connect(anaire_device_id.c_str())) {
      err_MQTT = false;
      Serial.println("MQTT connected");
      lastReconnectAttempt = 0;
      // Once connected resubscribe
      MQTT_client.subscribe(MQTT_receive_topic.c_str());
      Serial.print("MQTT connected - Receive topic: ");
      Serial.println(MQTT_receive_topic);
    } else {
      err_MQTT = true;
      Serial.print("failed, rc=");
      Serial.print(MQTT_client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}
void Send_Message_Cloud_App_MQTT() {  // Send measurements to the cloud application by MQTT
  // Print info
  Serial.print("Sending MQTT message to the send topic: ");
  Serial.println(MQTT_send_topic);
  sprintf(MQTT_message, "{id: %s,CO2: %d,humidity: %f,temperature: %f}", anaire_device_id.c_str(), (int) (CO2ppm_accumulated / CO2ppm_samples), humidity, temperature);
  Serial.print(MQTT_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents
  MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);
}
void Receive_Message_Cloud_App_MQTT(char* topic, byte* payload, unsigned int length) {  // callback function to receive configuration messages from the cloud application by MQTT
  boolean write_eeprom = false;       // to track if writing the eeprom is required
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

  // Update name
  if ((jsonBuffer["name"]) && (eepromConfig.anaire_device_name != jsonBuffer["name"])) {
    strncpy(eepromConfig.anaire_device_name, jsonBuffer["name"].as<const char*>(), sizeof(eepromConfig.anaire_device_name));
    eepromConfig.anaire_device_name[sizeof(eepromConfig.anaire_device_name) - 1] = '\0';
    write_eeprom = true;
    Serial.print("Anaire device name: ");
    Serial.println(eepromConfig.anaire_device_name);
  }

  // Update warning threshold
  if ((jsonBuffer["warning"]) && (eepromConfig.CO2ppm_warning_threshold != (int)jsonBuffer["warning"])) {
    eepromConfig.CO2ppm_warning_threshold = (int)jsonBuffer["warning"];
    Evaluate_CO2_Value();
    write_eeprom = true;
    Serial.print("New warning threshold: ");
    Serial.println(eepromConfig.CO2ppm_warning_threshold);
  }

  // Update alarm threshold
  if ((jsonBuffer["caution"]) && (eepromConfig.CO2ppm_alarm_threshold != (int)jsonBuffer["caution"])) {
    eepromConfig.CO2ppm_alarm_threshold = (int)jsonBuffer["caution"];
    Evaluate_CO2_Value();
    write_eeprom = true;
    Serial.print("New alarm threshold: ");
    Serial.println(eepromConfig.CO2ppm_alarm_threshold);
  }

  // Update sound alarm
  if ((jsonBuffer["alarm"]) && ((eepromConfig.sound_alarm) && (jsonBuffer["alarm"] == "OFF"))) {
    eepromConfig.sound_alarm = false;
    sound = false;
    write_eeprom = true;
    Serial.println("Alarm sound value: OFF");
  }

  if ((jsonBuffer["alarm"]) && ((!eepromConfig.sound_alarm) && (jsonBuffer["alarm"] == "ON"))) {
    eepromConfig.sound_alarm = true;
    sound = true;
    write_eeprom = true;
    Serial.println("Alarm sound value: ON");
  }

  // Check MQTT server
  if ((jsonBuffer["MQTT_server"]) && (eepromConfig.MQTT_server != jsonBuffer["MQTT_server"])) {
    strncpy(eepromConfig.MQTT_server, jsonBuffer["MQTT_server"], sizeof(eepromConfig.MQTT_server));
    eepromConfig.MQTT_server[sizeof(eepromConfig.MQTT_server) - 1] = '\0';
    write_eeprom = true;
    Serial.print("MQTT Server: ");
    Serial.println(eepromConfig.MQTT_server);

    //Attempt to connect to MQTT broker
    if (!err_wifi) {
      Init_MQTT();
    }
  }

  // Check MQTT port
  if ((jsonBuffer["MQTT_port"]) && (eepromConfig.MQTT_port != int(jsonBuffer["MQTT_port"]))) {
    eepromConfig.MQTT_port = int(jsonBuffer["MQTT_port"]);
    //strncpy(eepromConfig.MQTT_port, jsonBuffer["MQTT_port"], sizeof(eepromConfig.MQTT_port));
    //eepromConfig.MQTT_port[sizeof(eepromConfig.MQTT_port) - 1] = '\0';
    write_eeprom = true;
    Serial.print("MQTT Port: ");
    Serial.println(eepromConfig.MQTT_port);
    // Attempt to connect to MQTT broker
    if (!err_wifi) {
      Init_MQTT();
    }
  }

  // Check FRC value
  if ((jsonBuffer["FRC_value"]) && (eepromConfig.FRC_value != (uint16_t)jsonBuffer["FRC_value"])) {
    eepromConfig.FRC_value = (uint16_t)jsonBuffer["FRC_value"];
  }

  // Check temperature offset
  if ((jsonBuffer["temperature_offset"]) && (eepromConfig.temperature_offset != (uint16_t)jsonBuffer["temperature_offset"])) {
    eepromConfig.temperature_offset = (uint16_t)jsonBuffer["temperature_offset"];
    /*
    if (co2_sensor == MHZ14A) {
      //MHZ14A_Do_Temperature_Offset();
    }
    if (co2_sensor == SCD30) {
      SCD30_Do_Temperature_Offset();
    }
    */
  }

  // Check altitude_compensation
  if ((jsonBuffer["altitude_compensation"]) && (eepromConfig.altitude_compensation != (uint16_t)jsonBuffer["altitude_compensation"])) {
    eepromConfig.altitude_compensation = (uint16_t)jsonBuffer["altitude_compensation"];
    /*
    if (co2_sensor == scd30_sensor) {
      //SCD30_Do_Altitude_Compensation();
    }
    */
  }

  // If calibration has been enabled, justo do it
  if ((jsonBuffer["FRC"]) && (jsonBuffer["FRC"] == "ON")) {
    /*
    if (co2_sensor == scd30_sensor) {
      //Calibrate_SCD30();
    }
    */
  }

  // Update ABC Automatic Baseline Calibration
  if ((jsonBuffer["ABC"]) && ((eepromConfig.ABC) && (jsonBuffer["ABC"] == "OFF"))) {
    eepromConfig.ABC = false;
    write_eeprom = true;
    /*
    if (co2_sensor == scd30_sensor) {
      //SCD30_Do_AutoSelfCalibration();
    }
    */
    Serial.println("ABC: OFF");
  }

  if ((jsonBuffer["ABC"]) && ((!eepromConfig.ABC) && (jsonBuffer["ABC"] == "ON"))) {
    eepromConfig.ABC = true;
    write_eeprom = true;
    /*
    if (co2_sensor == scd30_sensor) {
      //SCD30_Do_AutoSelfCalibration();
    }
    */
    Serial.println("ABC: ON");
  }

  // if update flag has been enabled, wipe EEPROM and update to latest bin
  if (((jsonBuffer["update"]) && (jsonBuffer["update"] == "ON"))) {
    //boolean result = EEPROM.wipe();
    //if (result) {
    //  Serial.println("All EEPROM data wiped");
    //} else {
    //  Serial.println("EEPROM data could not be wiped from flash store");
    //}

    // update firmware to latest bin
    Serial.println("Update firmware to latest bin");
    //firmware_update();
  }

  // If factory reset has been enabled, just do it
  if ((jsonBuffer["factory_reset"]) && (jsonBuffer["factory_reset"] == "ON")) {
    Wipe_EEPROM ();   // Wipe EEPROM
    ESP.restart(); 
  }

  // If reboot, just do it, without cleaning the EEPROM
  if ((jsonBuffer["reboot"]) && (jsonBuffer["reboot"] == "ON")) {
    ESP.restart(); 
  }

  //print info
  Serial.println("MQTT update - message processed");
  Print_Config();

  // save the new values
  if (write_eeprom) {
    Write_EEPROM();
  }
}
void Setup_Sensors() {

  // Initialize Sensirion SCD30 driver
  Wire.begin(SCD30_SDA_pin, SCD30_SCL_pin);
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 chip");
  }
  else {
    Serial.println("SCD30 Found!");
    co2_sensor = scd30_sensor;
  }

  // if SCD30 is identified, get actual status
  if (co2_sensor == scd30_sensor) {
    if (scd30.selfCalibrationEnabled()) {
      Serial.println("Self calibration enabled");
    } else {
      Serial.println("Self calibration disabled");
    }
  }

}
void Read_Sensors() {

  // if SCD30 is identified
  if (co2_sensor == scd30_sensor) {
    if (scd30.dataReady()) {
      err_sensor = false;
      // Read SCD30
      if (!scd30.read()) {
        Serial.println("Error reading sensor data");
        return;
      }
      else {
        CO2ppm_value = scd30.CO2;
        temperature = scd30.temperature;
        humidity = scd30.relative_humidity;
      }
    }
    else {
      err_sensor = true;
      Serial.println("Error SCD30");
    }
  } // End SCD30 sensor

  if (!err_sensor) {
    // Provide the sensor values for Tools -> Serial Monitor or Serial Plotter
    Serial.print("CO2[ppm]:");
    Serial.print(CO2ppm_value);
    Serial.print("\t");
    Serial.print("Temperature[ºC]:");
    Serial.print(temperature, 1);
    Serial.print("\t");
    Serial.print("Humidity[%]:");
    Serial.println(humidity, 1);
    // Write measurements to bluetooth
    gadgetBle.writeCO2(CO2ppm_value);
    gadgetBle.writeTemperature(temperature);
    gadgetBle.writeHumidity(humidity);
    gadgetBle.commit();
  }
}
void Calibrate_Sensors() {
}
void Print_Config() {

  Serial.println("#######################################");
  Serial.print("device id: ");
  Serial.println(anaire_device_id);
  Serial.print("anaire device name: ");
  Serial.println(eepromConfig.anaire_device_name);
  Serial.print("SW version: ");
  Serial.println(sw_version);
  //Serial.print("WiFi SSID: ");
  //Serial.println(wifi_ssid);
  Serial.print("WiFi user: ");
  Serial.println(eepromConfig.wifi_user);
  Serial.print("WiFi password: ");
  Serial.println(eepromConfig.wifi_password);
  Serial.print("CO2ppm Warning threshold: ");
  Serial.println(eepromConfig.CO2ppm_warning_threshold);
  Serial.print("CO2ppm Alarm threshold: ");
  Serial.println(eepromConfig.CO2ppm_alarm_threshold);
  Serial.print("MQTT server: ");
  Serial.println(eepromConfig.MQTT_server);
  Serial.print("MQTT Port: ");
  Serial.println(eepromConfig.MQTT_port);
  Serial.print("Sound Alarm: ");
  Serial.println(eepromConfig.sound_alarm);
  Serial.print("Automatic Baseline Correction: ");
  Serial.println(eepromConfig.ABC);
  Serial.print("Forced Recalibration Value: ");
  Serial.println(eepromConfig.FRC_value);
  Serial.print("Temperature Offset: ");
  Serial.println(eepromConfig.temperature_offset);
  Serial.print("Altitude Compensation: ");
  Serial.println(eepromConfig.altitude_compensation);
  Serial.println("#######################################");

}
void espDelay(int ms) {  //! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}
void button_init() {

  // Top button click: show info and toggles sound
  button_top.setClickHandler([](Button2 & b) {
    Serial.println("Top button click: toggle sound");
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString(anaire_device_id, 10, 30);
    tft.drawString(sw_version, 10, 50);
    if (sound) {
      sound = false;
      tft.drawString("SIN sonido", 10, 120);
    }
    else {
      sound = true;
      tft.drawString("CON sonido", 10, 120);
    }

  });

  // Top button double click: activate SCD30 forced recalibration
  button_top.setDoubleClickHandler([](Button2 & b) {
    Serial.println("Top button double click: SCD30 calibration");
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("FORCED CALIBRATION", 10, 125);
    delay(5000);
    if (!scd30.forceRecalibrationWithReference(420)) {
      Serial.println("Failed to force recalibration at 420 ppm");
    }
    else {
      Serial.print("Forced recalibration performed at ");
      Serial.print(scd30.getForcedCalibrationReference());
      Serial.println(" ppm");
    }
    if (scd30.selfCalibrationEnabled()) {
      Serial.println("Self calibration enabled");
    } else {
      Serial.println("Self calibration disabled");
    }
    delay(1000);
  });

  // Top button triple click: activate SCD30 ABC auto calibration
  button_top.setTripleClickHandler([](Button2 & b) {
    Serial.println("Top button triple click: enable SCD30 self calibration");
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("AUTO CALIBRATION", 10, 125);
    delay(5000);
    if (!scd30.selfCalibrationEnabled(true)) {
      Serial.println("Failed to enable self calibration");
    }
    if (scd30.selfCalibrationEnabled()) {
      Serial.println("Self calibration enabled");
    } else {
      Serial.println("Self calibration disabled");
    }
    delay(1000);
  });

  // Bottom button click: sleep
  button_bottom.setClickHandler([](Button2 & b) {
    Serial.println("Bottom button click: sleep");
    int r = digitalRead(TFT_BL);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Press bottom button to wake up",  tft.width() / 2, tft.height() / 2 );
    espDelay(6000);
    digitalWrite(TFT_BL, !r);
    tft.writecommand(TFT_DISPOFF);
    tft.writecommand(TFT_SLPIN);
    //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    // esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW);
    // set bottom button for wake up
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    delay(200);
    esp_deep_sleep_start();
  });

  // Bottom button double click: restart
  button_bottom.setDoubleClickHandler([](Button2 & b) {
    Serial.println("Bottom button double click: restart");
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("RESTART", 10, 125);
    ESP.restart();
  });

  // Bottom button triple click: launch captive portal to configure WiFi and MQTT
  button_bottom.setTripleClickHandler([](Button2 & b) {
    Serial.println("Bottom button triple click: captive portal");
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("CONFIG AnaireWiFi", 10, 125);

    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;
    wifiManager.setDebugOutput(true);
    wifiManager.setCountry("ES");
    wifiManager.disconnect();
    WiFi.mode(WIFI_AP); // explicitly set mode, esp defaults to STA+AP

    /*
      // Captive portal parameters
      WiFiManagerParameter custom_wifi_html("<p>Set WPA2 Enterprise</p>"); // only custom html
      WiFiManagerParameter custom_wifi_user("User", "WPA2 Enterprise user", eepromConfig.wifi_user, 24);
      WiFiManagerParameter custom_wifi_password("Password", "WPA2 Enterprise Password", eepromConfig.wifi_password, 24);
      WiFiManagerParameter custom_mqtt_html("<p>Set MQTT server</p>"); // only custom html
      WiFiManagerParameter custom_mqtt_server("Server", "MQTT server", eepromConfig.MQTT_server, 24);
      char port[6]; itoa(eepromConfig.MQTT_port, port, 10);
      WiFiManagerParameter custom_mqtt_port("Port", "MQTT port", port, 6);

      //wifiManager.setSaveParamsCallback(saveParamCallback);

      // Add parameters
      wifiManager.addParameter(&custom_wifi_html);
      wifiManager.addParameter(&custom_wifi_user);
      wifiManager.addParameter(&custom_wifi_password);
      wifiManager.addParameter(&custom_mqtt_html);
      wifiManager.addParameter(&custom_mqtt_server);
      wifiManager.addParameter(&custom_mqtt_port);
    */

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    wifiManager.setConfigPortalTimeout(600);

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    wifiManager.resetSettings(); // reset previous configurations
    bool res = wifiManager.startConfigPortal("AnaireWIFI");
    if (!res) {
      Serial.println("Not able to start captive portal");
    } else {
      //if you get here you have connected to the WiFi
      Serial.println("Captive portal operative");
    }

    /*
      // Save parameters to EEPROM only if any of them changed
      bool write_eeprom = false;

      if (eepromConfig.wifi_user != custom_wifi_user.getValue()) {
      strncpy(eepromConfig.wifi_user, custom_wifi_user.getValue(), sizeof(eepromConfig.wifi_user));
      eepromConfig.wifi_user[sizeof(eepromConfig.wifi_user) - 1] = '\0';
      write_eeprom = true;
      Serial.print("WiFi user: ");
      Serial.println(eepromConfig.wifi_user);
      }
      if (eepromConfig.wifi_password != custom_wifi_password.getValue()) {
      strncpy(eepromConfig.wifi_password, custom_wifi_password.getValue(), sizeof(eepromConfig.wifi_password));
      eepromConfig.wifi_password[sizeof(eepromConfig.wifi_password) - 1] = '\0';
      write_eeprom = true;
      Serial.print("WiFi password: ");
      Serial.println(eepromConfig.wifi_password);
      }
      if (eepromConfig.MQTT_server != custom_mqtt_server.getValue()) {
      strncpy(eepromConfig.MQTT_server, custom_mqtt_server.getValue(), sizeof(eepromConfig.MQTT_server));
      eepromConfig.MQTT_server[sizeof(eepromConfig.MQTT_server) - 1] = '\0';
      write_eeprom = true;
      Serial.print("MQTT server: ");
      Serial.println(eepromConfig.MQTT_server);
      }

      if (eepromConfig.MQTT_port != atoi(custom_mqtt_port.getValue())) {
      eepromConfig.MQTT_port = atoi(custom_mqtt_port.getValue());
      write_eeprom = true;
      Serial.print("MQTT port: ");
      Serial.println(eepromConfig.MQTT_port);
      }

      if (write_eeprom) {
      Write_EEPROM();
      }
    */

    // Restart
    //ESP.restart();

  });

}
void button_loop() {
  button_top.loop();
  button_bottom.loop();
}
void displayInit() {
  tft.init();
  tft.setRotation(1);
}
void displaySplashScreen() {
  tft.pushImage(0, 0,  240, 135, anaire_ttgo_splash);
}
void displayCo2(uint16_t co2, float temp, float hum) {

  if (co2 > 9999) {
    co2 = 9999;
  }

  uint8_t defaultDatum = tft.getTextDatum();

  // Set screen and text colours based on CO2 value
  if (co2 >= 1000 ) {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    if (sound) {
      digitalWrite(BUZZER_GPIO, HIGH);
    }
    delay(1000);
    digitalWrite(BUZZER_GPIO, LOW);
  } else if (co2 >= 700 ) {
    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_RED, TFT_YELLOW);
    if (sound) {
      digitalWrite(BUZZER_GPIO, HIGH);
    }
    delay(100);
    digitalWrite(BUZZER_GPIO, LOW);
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    digitalWrite(BUZZER_GPIO, LOW);
  }

  // Draw CO2 number
  tft.setTextDatum(8); // bottom right
  tft.setTextSize(1);
  tft.setFreeFont(FF95);
  tft.drawString(String(co2), 195, 105);

  // Draw CO2 units
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  tft.drawString("ppm", 230, 90);

  // Draw temperature
  tft.setTextDatum(6); // bottom left
  tft.drawString(String(temp), 10, 125);
  tft.drawString("C", 50, 125);

  // Draw humidity
  tft.drawString(String(hum), 90, 125);
  tft.drawString("%", 130, 125);

  // Draw bluetooth device id
  tft.setTextDatum(8); // bottom right
  tft.drawString(gadgetBle.getDeviceIdString(), 230, 125);

  // Revert datum setting
  tft.setTextDatum(defaultDatum);

}

void Get_Anaire_DeviceId() {
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  anaire_device_id = String(chipId, HEX); // HEX format for backwards compatibility to Anaire devices based on NodeMCU board
  Serial.printf("ESP32 Chip model = %s Rev %d.\t", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores and %dMB Flash.\n", ESP.getChipCores(), ESP.getFlashChipSize() / (1024 * 1024));
  Serial.print("Anaire Device ID: ");
  Serial.println(anaire_device_id);
}

void Evaluate_CO2_Value() {

  // Recovering to "ok" status stops any warning or alarm warnings and quits special mode (after pressing flash button), therefore after passing by "ok" status
  // the device is "reset" and when entering warning or alarm state the device will report localy again by blinking o2_builtin_led_gpio16 led and the buzzer,
  // and pushing the flash button will be required if the user wants to stop light and sound alerting

  if (CO2ppm_value < eepromConfig.CO2ppm_warning_threshold) {
    co2_device_status = co2_ok;                             // update co2 status
    alarm_ack = false;                                  // Init alarm ack status
  }

  // Status: warning
  else if ((CO2ppm_value >= eepromConfig.CO2ppm_warning_threshold) && (CO2ppm_value < eepromConfig.CO2ppm_alarm_threshold)) {
    co2_device_status = co2_warning;                  // update device status
  }

  // Status: alarm
  else {
    co2_device_status = co2_alarm; // update device status
    //if ((!alarm_ack) && (eepromConfig.sound_alarm)) {                                   // flash button hasn't been pressed to disable and sound_alarm is enabled
    //  blinker_BUZZER_GPIO.attach_ms(ALARM_BLINK_PERIOD, changeState_BUZZER_GPIO);       // alarm sound on buzzer
    //}
  }

  // Print info on serial monitor
  switch (co2_device_status) {
    case co2_ok:
      Serial.println ("STATUS: CO2 OK");
      break;
    case co2_warning:
      Serial.println ("STATUS: CO2 WARNING");
      break;
    case co2_alarm:
      Serial.println ("STATUS: CO2 ALARM");
      break;
  }

  // Update display with new values
  displayCo2((uint16_t) round(scd30.CO2), scd30.temperature, scd30.relative_humidity);

}

void Read_EEPROM() {
  if (preferences.getBytesLength("config") > 0) {
    boolean result = preferences.getBytes("config", &eepromConfig, sizeof(eepromConfig));
    if (result) {
      Serial.println("Config data read from flash");
    } else {
      Serial.println("Config data could not be read from flash");
    }
  }
  else {
    anaire_device_id.toCharArray(eepromConfig.anaire_device_name, sizeof(eepromConfig.anaire_device_name)); // Initialize anaire_device_name with anaire_device_id
    Serial.println("No EEPROM data - using default config values");
  }

}

void Write_EEPROM() {
  boolean result = preferences.putBytes("config", &eepromConfig, sizeof(eepromConfig));
  if (result) {
    Serial.println("Config data written to flash");
  } else {
    Serial.println("Config data could not be written to flash");
  }
}

void Wipe_EEPROM() {
  boolean result = preferences.clear();
  if (result) {
    Serial.println("All EEPROM data wiped");
  } else {
    Serial.println("EEPROM data could not be wiped from flash store");
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(115200);
  while (!Serial) {
    delay(500); // wait 0.5 seconds for connection
  }
  Serial.setDebugOutput(true);

  // print info
  Serial.println();
  Serial.println("### INIT ANAIRE PiCO2 DEVICE ###########################################");

  // Initialize TTGO Display and show Anaire splash screen
  displayInit();
  displaySplashScreen();
  delay(3000); // Enjoy the splash screen for 3 seconds

  // init preferences to handle persitent config data
  preferences.begin("config"); // use "config" namespace

  // Get device id
  Get_Anaire_DeviceId();
  
  // Set MQTT topics
  MQTT_send_topic = "measurement";
  MQTT_receive_topic = "config/" + anaire_device_id;  // config messages will be received in config/id

  // Read EEPROM config values
  //Read_EEPROM();
  Print_Config();

  // Initialize the GadgetBle Library for Bluetooth
  gadgetBle.begin();
  Serial.print("Sensirion GadgetBle Lib initialized with deviceId = ");
  Serial.println(gadgetBle.getDeviceIdString());

  // Initialize buzzer to OFF
  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, LOW);

  // Initialize TTGO board buttons
  button_init();

  // Attempt to connect to WiFi network:
  Connect_WiFi();

  // Attempt to connect to MQTT broker
  if (!err_wifi) {
    Init_MQTT();
  }

  // Initialize and warm up device sensors
  Setup_Sensors();

  // Init loops
  measurements_loop_start = millis();
  MQTT_loop_start = millis();
  errors_loop_start = millis();

}
///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  /*
    // If a firmware update is in progress do not do anything else
    if (!updating) {
  */

  //Serial.println ("--- LOOP BEGIN ---");

  // Measurement loop
  if ((millis() - measurements_loop_start) >= measurements_loop_duration)
  {

    // New timestamp for the loop start time
    measurements_loop_start = millis();

    // Read sensors
    Read_Sensors();

    // Evaluate CO2 value
    Evaluate_CO2_Value();

    // Accumulates samples
    CO2ppm_accumulated += CO2ppm_value;
    CO2ppm_samples++;

  }

  // MQTT loop
  if ((millis() - MQTT_loop_start) >= MQTT_loop_duration)
  {

    // New timestamp for the loop start time
    MQTT_loop_start = millis();

    // Message the MQTT broker in the cloud app to send the measured values
    if (!err_wifi) {
      Send_Message_Cloud_App_MQTT();
    }

    // Reset samples after sending them to the MQTT server
    CO2ppm_accumulated = 0;
    CO2ppm_samples = 0;

  }

  // Errors loop
  if ((millis() - errors_loop_start) >= errors_loop_duration)
  {

    // New timestamp for the loop start time
    errors_loop_start = millis();

    // Try to recover error conditions
    if (err_sensor) {
      Serial.println ("--- err_sensor");
      Setup_Sensors();  // Init co2 sensors
    }

    /*
      if ((err_wifi) || (WiFi.status() != WL_CONNECTED)) {
      Serial.println ("--- err_wifi");
      err_wifi = true;
      Connect_WiFi();   // Attempt to connect to WiFi network:
      }

      //Reconnect MQTT if needed
      if ((!MQTT_client.connected()) && (!err_wifi)) {
      Serial.println ("--- err_mqtt");
      err_MQTT = true;
      }
    */

    //Reconnect MQTT if needed
    if ((err_MQTT) && (!err_wifi)) {
      Serial.println ("--- MQTT reconnect");
      // Attempt to connect to MQTT broker
      //MQTTReconnect();
      Init_MQTT();
    }

    // if not there are not connectivity errors, receive MQTT messages
    if ((!err_MQTT) && (!err_wifi)) {
      //MQTT_client.loop();
    }

  }

  /*
    // if FLASH button was pressed flag to updated OLED was set; the display cannot be updated from the ISR, but from main loop after this flag is set
    if (((update_OLED_co2_flag) && ((millis() - flash_button_press_start)) > 3000)) { // do it only if button was pressed more than 3s ago
      update_OLED_CO2();
      update_OLED_co2_flag = false; // reset flag
    }

    // if FLASH button was pressed flag to updated OLED was set; the display cannot be updated from the ISR, but from main loop after this flag is set
    if (update_OLED_status_flag) {
      update_OLED_Status();
      update_OLED_status_flag = false; // reset flag
    }

    // if the flash button was pressed more than 10 seconds ago and it is still low, launch calibration
    if (digitalRead(FLASH_BUTTON_GPIO)) { // button not pressed
      flash_button_pressed_flag = false; // reset flash button press start
    }
    else if ((millis() - flash_button_press_start) > 10000) {   // button is pressed since >10s
      if (co2_sensor == MHZ14A) {
        Calibrate_MHZ14A();
      }
      if (co2_sensor == SCD30) {
        Calibrate_SCD30();
      }
    }

    // Call the double reset detector loop method every so often,so that it can recognise when the timeout expires.
    // You can also call drd.stop() when you wish to no longer consider the next reset as a double reset.
    drd.loop();

  */

  // Process wifi server requests
  Check_WiFi_Server();

  // Process bluetooth events
  gadgetBle.handleEvents();

  // Process buttons events
  button_loop();

  //Serial.println("--- END LOOP");
}
