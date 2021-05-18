//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 20201109 ANAIRE DEVICE CODE anaire.org anaire@anaire.org
//
// Anaire code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT)
//
// Get CO2, temperature and humidity measurements and send them to the Anaire Cloud Application https://github.com/anaireorg/anaire-cloud
// Show local measurements and provide local warning and alarm visual and sound indications
//
// Parts - Common platform:
//   Control board: AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
//   Buzzer: AZDelivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
//   Display: AZDelivery 0.91 inch OLED I2C Display 128 x 32 Pixels  https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display
//
// Parts - Sensors:
//   Option 1, better measurements precission: Sensirion SCD30 for CO2, temperature and humidity https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-co2/
//   Option 2, less cost: MHZ14A for CO2 http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf and AZ-Delivery DHT family Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul / https://www.az-delivery.de/es/products/dht22-temperatursensor-modul
//
// Arduino IDE Setup:

//   Install the usb to uart driver from https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
//   Start Arduino and open Preferences window.
//   Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
//   Open Boards Manager from Tools > Board menu and find esp8266 platform by esp8266 community and install the software for Arduino from a drop-down box.
//   Select "NodeMCU 1.0" board from Tools > Board menu after installation
//
// Install the following libraries in Arduino IDE:
//   WiFiEsp by bportaluri https://github.com/bportaluri/WiFiEsp
//   Pubsubclient by Nick O'Leary https://pubsubclient.knolleary.net/
//   ArduinoJson by Benoit Blanchon https://arduinojson.org/?utm_source=meta&utm_medium=library.properties
//   esp8266-oled-ssd1306 by ThingPulse, Fabrice Weinberg https://github.com/ThingPulse/esp8266-oled-ssd1306
//   DHTesp by beegee_tokyo https://github.com/beegee-tokyo/DHTesp
//   ESP_EEPROM by j-watson https://github.com/jwrw/ESP_EEPROM
//   WifiManager by tzapu,tablatronix https://github.com/tzapu/WiFiManager
//   Double Reset detector by Stephen Denne https://github.com/datacute/DoubleResetDetector

//   MHZ-Z19 Library from WifWaf https://github.com/WifWaf/MH-Z19 Library - to manage Winsen CO2 sensors - INSTALL FROM ZIP FILE with Sketch-> Include Library-> Add .ZIP library
//   Modified paulvha SCD30 library, get it from anaire github on https://github.com/anaireorg/anaire-devices/blob/main/src/scd30-master.zip - INSTALL FROM ZIP FILE with Sketch-> Include Library-> Add .ZIP library

// Design leads:

// - The ID (last 3 fields from MAC address) and IP address are shown on OLED display during boot and after pressing the Flash button
// - Pressing the reset button ("R") twice in less than 10 seconds restarts the device in a captive web configuration portal, useful to configure local wifi settings and MQTT endpoint
// - All other local config parameters (like name, thresholds, local alarm, etc.) are configured via the cloud app, after connecting
// - Pressing Alarm button ("A") toggles between activating/deactivating local sound alarm alarms, and also show device information on the display
//   * Stops local sound alerting until pressed again or until CO2 level decays below the warning threshold, deactivating therefore local alerting and reseting the local alerting system
//   * It also shows device ID and IP address after being pressed during 3 seconds
// - Pressing Alarm button ("A") during more than 10s activates forced calibration
// - Firmware update with latest fimrware available though cloud app
//   * Always using latest firmware on https://github.com/anaireorg/anaire-devices/blob/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin
// - Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector):
//   * When CO2 Status is "ok" (below warning threshold) LED and buzzer are off (normal status)
//   * When CO2 Status is "warning" builtin LED and external buzzer alternate blink at a slow pace (WARNING_BLINK_PERIOD)
//   * When CO2 Status is "alarm" builtin LED and external buzzer alternate blink at fast pace (ALARM_BLINK_PERIOD)
// - Two options for sensors:
//   * Either Sensirion SCD30 for CO2, temperature and humidity
//   * Or MH-Z14A for CO2, and DHT11 or DHT22 for temperature and humidity
// - The device is designed to work only with the CO2 sensor, so buzzer, DHT humidity and temperature sensor, and OLED display are optional
// - The device is designed to recover from Wifi, MQTT or sensors reading temporal failures. Local measurements will always be shown in the local display
// - The web server is activated, therefore entering the IP on a browser allows to see device specific details and measurements; device forced calibration is also available through the web server

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String sw_version = "v2.20210307.Deo";
// 20210307 Fixed remoted updates by reducing BearSSL buffer size
// 20210304 Troubleshooting remote updates after enabling serial debug; display modified to show ppm in 16p before ppm value on 24p font
// 20210228 Fixed execution of individual MQTT commands; firmware updates work if Wifi connection is fast
// 20210223 Fixed MQTT error problem when Wifi didn't connect on the first try
// 20210221 Range of Winsen MH-Z14A/MH-Z19c set up to 2000ppm as it is enough to secure environments against COVID and provides more accuracy
// 20210221 The CO2 measurement sent by MQTT every MQTT loop (30s) is calculated as the mean of the measured values during the measurement loop (5s)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen
// i.e: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String anaire_device_id = String(ESP.getChipId(), HEX);   // HEX version, for easier match to mac address

// The neatest way to access variables stored in EEPROM is using a structure
// Init to default values; if they have been chaged they will be readed later, on initialization
struct MyEEPROMStruct {
  char anaire_device_name[24] = "";                           // Device name; default to anaire_device_id
  uint16_t CO2ppm_warning_threshold = 700;                    // Warning threshold; default to 700ppm
  uint16_t CO2ppm_alarm_threshold = 1000;                     // Alarm threshold; default to 1000ppm
  char MQTT_server[24] = "mqtt.anaire.org";                   // MQTT server url or public IP address. Default to Anaire Portal on portal.anaire.org
  uint16_t MQTT_port = 80;                                    // MQTT port; Default to Anaire Port on 30183
  boolean sound_alarm = true;                                 // Global flag to control sound alarm; default to true
  boolean ABC = false;                                        // Automatic baseline Correction; default to false
  uint16_t FRC_value = 400;                                   // Forced ReCalibration value; default to 400ppm
  uint16_t temperature_offset = 0;                            // temperature offset for SCD30 CO2 measurements
  uint16_t altitude_compensation = 0;                         // altitude compensation for SCD30 CO2 measurements
  char wifi_user[24] = "";                                    // WiFi user to be used on WPA Enterprise. Default to null (not used)
  char wifi_password[24] = "";                                // WiFi password to be used on WPA Enterprise. Default to null (not used)
} eepromConfig;

// Save config values to EEPROM
#include <ESP_EEPROM.h>

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

// flash button duration: time since flash button was pressed
unsigned long flash_button_press_start;            // holds a timestamp for each control loop start

// CO2 Blinking period, used to reflect CO2 status on builtin led and buzzer
const int WARNING_BLINK_PERIOD = 1000;            // 1 second
const int ALARM_BLINK_PERIOD = 200;               // 0.2 seconds

// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// WiFi
#include <ESP8266WiFi.h>                          // Wifi ESP8266
extern "C" {
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "c_types.h"
}

#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>                         // to be reached on anaire_device_id.local in the local network
WiFiClient wifi_client;
const int WIFI_CONNECT_TIMEOUT = 5000;           // 5 seconds
int wifi_status = WL_IDLE_STATUS;
WiFiServer wifi_server(80);                      // to check if it is alive
String wifi_ssid = WiFi.SSID();                  // your network SSID (name)
String wifi_password = WiFi.psk();               // your network psk password

#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <DoubleResetDetector.h>

// Number of seconds after reset during which a subseqent reset will be considered a double reset.
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
#define OLED_SCK_GPIO 14  // signal GPIO14 (D5)
#define OLED_SDA_GPIO 12  // signal GPIO12 (D6)

// for 128x32 displays:
SSD1306Wire display(0x3c, OLED_SDA_GPIO, OLED_SCK_GPIO, GEOMETRY_128_32);  // ADDRESS, SDA, SCL

// MQTT
#include <PubSubClient.h>
char MQTT_message[256];
String MQTT_send_topic = "measurement";
String MQTT_receive_topic = "config/" + anaire_device_id;  // config messages will be received in config/id
PubSubClient MQTT_client(wifi_client);
char received_payload[384];

//JSON
#include <ArduinoJson.h>
StaticJsonDocument<384> jsonBuffer;

// Sensirion SCD CO2, temperature and humidity sensor
#define SCD30WIRE Wire
#include "paulvha_SCD30.h"
#define SCD30_SCK_GPIO 14 // signal GPIO14 (D5)
#define SCD30_SDA_GPIO 12 // signal GPIO12 (D6)

SCD30 airSensor;
unsigned long SCD30_WARMING_TIME = 2000;                                // SCD30 CO2 sensor warming time
unsigned long SCD30_CALIBRATION_TIME = 180000;                          // SCD30 CO2 CALIBRATION TIME: 3 min = 180000 ms
uint16_t SCD30_MEASUREMENT_INTERVAL = measurements_loop_duration / 1000; // time between measurements

// MHZ14A CO2 sensor: software serial port
#include "SoftwareSerial.h"                            // Remove if using HardwareSerial or non-uno compatabile device
#include "MHZ19.h"                                     // https://github.com/WifWaf/MH-Z19 Library
MHZ19 myMHZ19;
#define MHZ_BAUDRATE 9600                              // Native to the sensor (do not change)
const unsigned long MHZ14A_WARMING_TIME = 180000;      // MHZ14A CO2 sensor warming time: 3 minutes = 180000 ms
const unsigned long MHZ14A_SERIAL_TIMEOUT = 3000;      // MHZ14A CO2 serial start timeout: 3 seconds = 3000 ms
const unsigned long MHZ14A_CALIBRATION_TIME = 1200000; // MHZ14A CO2 CALIBRATION TIME: 20 min = 1200000 ms
#define swSerialRX_gpio 13
#define swSerialTX_gpio 15
SoftwareSerial mySerial(swSerialRX_gpio, swSerialTX_gpio);
byte measurement_command[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // Command to get measurements from MHZ14A CO2 sensor
byte calibration_command[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // Command to calibrate MHZ14A CO2 sensor
char response_CO2[9];   // holds the received data from MHZ14A CO2 sensor
int response_CO2_high;  // holds upper byte
int response_CO2_low;   // holds lower byte

int CO2ppm_value = 0;         // CO2 ppm measured value
int CO2ppm_accumulated = 0;   // Accumulates co2 measurements for a MQTT period
int CO2ppm_samples = 0;       // Counts de number of samples for a MQTT period

// AZ-Delivery DHT11/DHT22
#include "DHTesp.h"
#define DHT_GPIO 5      // signal GPIO5 (D1)

// Initialize DHT sensor
DHTesp dht;
float temperature;      // Read temperature as Celsius
float humidity;         // Read humidity in %

// AZ-Delivery Active Buzzer
#define BUZZER_GPIO 4   // signal GPIO4 (D2)

// GPIO0 (D3) is button FLASH on nodemcu PCB
#define FLASH_BUTTON_GPIO 0

// NodeMCU Builtin LED, used to provide CO2 visual status info:
// CO2 OK: off
// WARNING: blinks slow
// ALARM: blinks fast
#define CO2_STATUS_BUILTIN_LED_GPIO 16                            // GPIO16 (D0), closer to the usb port, on the NodeMCU PCB - below the button labeled R (Reset) on 3D Box
//#define CO2_STATUS_BUILTIN_LED_GPIO 2                           // GPIO2 (D4), on the ESP-12 module’s PCB, far away from the usb port - near button labeled A (Alarm) on 3D Box

// The other NodeMCU builtin LED, used to provide device status info:
// Device OK: off
// Any error or config status: on
//#define DEVICE_STATUS_BUILTIN_LED_GPIO 16                       // GPIO16 (D0), closer to the usb port, on the NodeMCU PCB -  below the button labeled R (Reset) on 3D Box
#define DEVICE_STATUS_BUILTIN_LED_GPIO 2                        // GPIO2 (D4), on the ESP-12 module’s PCB, far away from the usb port - near button labeled A (Alarm) on 3D Box

// CO2 sensors
enum CO2_sensors {none, MHZ14A, SCD30}; // possible sensors integrated in the SW
CO2_sensors co2_sensor = none;

// CO2 device status
enum CO2_status {ok, warning, alarm}; // the device can have one of those CO2 status
CO2_status co2_device_status = ok;        // initialized to ok

// device status
boolean err_global = false;
boolean err_wifi = false;
boolean err_MQTT = false;
boolean err_co2 = false;
boolean err_dht = false;
boolean err_oled = false;

// Ticker library to blink leds and buzzer
#include <Ticker.h>  //Ticker Library
Ticker blinker_CO2_STATUS_BUILTIN_LED_GPIO;       // to blink CO2_STATUS_BUILTIN_LED_GPIO
Ticker blinker_BUZZER_GPIO;                       // to blink BUZZER_GPIO for the alarm sound

// flag to update OLED display with status info from main loop instead of button ISR
boolean update_OLED_status_flag = false;

// flag to update OLED display with CO2 info from main loop instead of button ISR
boolean update_OLED_co2_flag = false;

// flag to store flash button status
boolean flash_button_pressed_flag = false;

// to indicate if push button has been pushed to ack the alarm and switch off the buzzer
boolean alarm_ack = false;

// to know when there is an updating process in place
boolean updating = false;

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(115200);
  while (!Serial) {
    Serial.print("Attempting to open monitor serial port ");
    delay(1000); // wait 1 seconds for connection
  }

  // Enable debug
  Serial.setDebugOutput(true);
  
  // print info
  Serial.println();
  Serial.println();
  Serial.println("### INIT ANAIRE DEVICE ###########################################");

  // Initialize DEVICE_STATUS_BUILTIN_LED to ON while the device is initializing
  pinMode(DEVICE_STATUS_BUILTIN_LED_GPIO, OUTPUT);
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

  // Initialize CO2_STATUS_BUILTIN_LED to OFF
  pinMode(CO2_STATUS_BUILTIN_LED_GPIO, OUTPUT);
  digitalWrite(CO2_STATUS_BUILTIN_LED_GPIO, HIGH);

  // Initialize BUZZER to OFF
  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, LOW);

  // Push button: attach interrupt to togle sound alarm and provide device info on the display
  // It will be used also to launch forced calibration
  pinMode(FLASH_BUTTON_GPIO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLASH_BUTTON_GPIO), push_button_handler, FALLING);

  // Init OLED display
  display.init();

  // Read EEPROM config values
  Read_EEPROM();

  // Print config
  Print_Config();

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

    // Device status LED ON
    digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

    // Update display to inform
    display.flipScreenVertically();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "CONFIG");
    display.drawString(0, 16, "ESP_" + String(anaire_device_id));
    display.display(); // update OLED display

    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

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

    // Device status LED OFF
    digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);

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
      //strncpy(eepromConfig.MQTT_port, custom_mqtt_port.getValue(), sizeof(eepromConfig.MQTT_port));
      //eepromConfig.MQTT_port[sizeof(eepromConfig.MQTT_port) - 1] ='\0';
      write_eeprom = true;
      Serial.print("MQTT port: ");
      Serial.println(eepromConfig.MQTT_port);
    }

    if (write_eeprom) {
      Write_EEPROM();
    }

    // Reset
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up

    // so resetting the device allows to go back into config mode again when it reboots.
    delay(1000);

  }

  // Normal start, without captive portal
  else {

    // Put anaire.org in the display
    display.flipScreenVertically();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_24);
    display.drawString(64, 4, "anaire.org");
    display.display();                      // update OLED display
    delay(1000);                            // to show anaire.org
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, String(anaire_device_id));
    display.drawString(0, 16, sw_version);
    display.display(); // update OLED display
    delay(1000);

    // Attempt to connect to WiFi network:
    Connect_WiFi();

    // Attempt to connect to MQTT broker
    if (!err_wifi) {
      Init_MQTT();
    }

    // Initialize and warm up device sensors
    Setup_sensors();

    // Init loops
    measurements_loop_start = millis();
    MQTT_loop_start = millis();
    errors_loop_start = millis();

    // Set DEVICE_STATUS_BUILTIN_LED to OFF when finishing initializing
    digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // If a firmware update is in progress do not do anything else
  if (!updating) {

    //Serial.println ("--- LOOP BEGIN ---");

    // Measurement loop
    if ((millis() - measurements_loop_start) >= measurements_loop_duration)
    {

      // New timestamp for the loop start time
      measurements_loop_start = millis();

      // Read sensors
      Read_Sensors();

      // Evaluate CO2 value
      Evaluate_CO2_Value(); // this is to avoud to refresh the display with the las co2 measurement value

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

      // Reset samples
      CO2ppm_accumulated = 0;
      CO2ppm_samples = 0;

    }

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

    // Process wifi server requests
    Check_WiFi_Server();

    // Errors loop
    if ((millis() - errors_loop_start) >= errors_loop_duration)
    {

      // New timestamp for the loop start time
      errors_loop_start = millis();

      // Try to recover error conditions
      if (err_co2) {
        Serial.println ("--- err_co2");
        Setup_sensors();  // Init co2 sensors
      }

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

      //Reconnect MQTT if needed
      if ((err_MQTT) && (!err_wifi)) {
        Serial.println ("--- MQTT reconnect");
        // Attempt to connect to MQTT broker
        //MQTTReconnect();
        Init_MQTT();
      }

      // if not there are not connectivity errors, receive MQTT messages
      if ((!err_MQTT) && (!err_wifi)) {
        MQTT_client.loop();
      }

    }

    //Serial.println("--- END LOOP");
    //Serial.println();

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

// Connect to WiFi network
void Connect_WiFi() {

  Serial.print("Attempting to connect to Network named: ");
  Serial.println(wifi_ssid); // print the network name (SSID);

  // Set wifi mode
  WiFi.mode(WIFI_STA);

  // If there are not wifi user and wifi password defined, proceed to traight forward configuration
  if ((strlen(eepromConfig.wifi_user) == 0) && (strlen(eepromConfig.wifi_password) == 0)) {

    //WiFi.begin(wifi_ssid, wifi_password);
    WiFi.begin();

  }

  else {  // set up wpa2 enterprise

    Serial.print("Attempting to authenticate with WPA Enterprise ");
    Serial.print("User: ");
    Serial.println(eepromConfig.wifi_user);
    Serial.print("Password: ");
    Serial.println(eepromConfig.wifi_password);

    // Setting ESP into STATION mode only (no AP mode or dual mode)
    wifi_set_opmode(STATION_MODE);

    struct station_config wifi_config;

    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char*)wifi_config.ssid, wifi_ssid.c_str());
    strcpy((char*)wifi_config.password, wifi_password.c_str());

    wifi_station_set_config(&wifi_config);
    //uint8_t target_esp_mac[6] = {0x24, 0x0a, 0xc4, 0x9a, 0x58, 0x28};
    //wifi_set_macaddr(STATION_IF,target_esp_mac);
    wifi_station_set_wpa2_enterprise_auth(1);

    // Clean up to be sure no old data is still inside
    wifi_station_clear_cert_key();
    wifi_station_clear_enterprise_ca_cert();
    wifi_station_clear_enterprise_identity();
    wifi_station_clear_enterprise_username();
    wifi_station_clear_enterprise_password();
    wifi_station_clear_enterprise_new_password();

    // Set up authentication
    //wifi_station_set_enterprise_identity((uint8*)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_username((uint8*)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_password((uint8*)eepromConfig.wifi_password, strlen((char*)eepromConfig.wifi_password));

    wifi_station_connect();

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
  }
  else {
    err_wifi = false;

    wifi_server.begin(); // start the web server on port 80
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Print your WiFi shield's MAC address:
    Serial.print("MAC Adress: ");
    Serial.println(WiFi.macAddress());

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
  Serial.print("Attempting to connect to the MQTT broker ");
  Serial.print(eepromConfig.MQTT_server);
  Serial.print(":");
  Serial.println(eepromConfig.MQTT_port);

  // Attempt to connect to MQTT broker
  MQTT_client.setBufferSize(512); // to receive messages up to 512 bytes length (default is 256)
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
            client.print("Anaire Device ID: ");
            client.print(anaire_device_id);
            client.println("<br>");
            client.print("Anaire Device name: ");
            client.print(eepromConfig.anaire_device_name);
            client.println("<br>");
            client.print("SW version: ");
            client.print(sw_version);
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
            if (co2_sensor == SCD30) {
              client.print("CO2 Sensor: Sensirion SCD30");
              client.println("<br>");
              uint16_t val;
              //SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO, false);
              airSensor.getMeasurementInterval(&val);
              client.print("SCD30 Measurement Interval: ");
              client.print(val);
              client.println("<br>");
              client.print("SCD30 AutoselfCalibration: ");
              client.print(airSensor.getAutoSelfCalibration());
              client.println("<br>");
              airSensor.getForceRecalibration(&val);
              client.print("SCD30 Force Recalibration: ");
              client.print(val);
              client.println("<br>");
              airSensor.getTemperatureOffset(&val);
              client.print("SCD30 Temperature Offset: ");
              client.print(val);
              client.println("<br>");
              airSensor.getAltitudeCompensation(&val);
              client.print("SCD30 AltitudeCompensation: ");
              client.print(val);
              client.println("<br>");
            }
            else {
              client.print("CO2 Sensor: Winsen MH-Z14A/Z19x");
              client.println("<br>");
              char myVersion[4];
              myMHZ19.getVersion(myVersion);
              client.print("\nFirmware Version: ");
              for (byte i = 0; i < 4; i++)
              {
                client.print(myVersion[i]);
                if (i == 1)
                  client.print(".");
              }
              client.println("<br>");
              client.print("Range: ");
              client.println(myMHZ19.getRange());
              client.println("<br>");
              client.print("Accuracy: ");
              client.println(myMHZ19.getAccuracy(true));   // force to get the value, even if nor available
              client.println("<br>");
              client.print("Background CO2: ");
              client.println(myMHZ19.getBackgroundCO2());
              client.println("<br>");
              client.print("Temperature Cal: ");
              client.println(myMHZ19.getTempAdjustment());
              client.println("<br>");
              client.print("ABC Status: "); myMHZ19.getABC() ? client.println("ON") :  client.println("OFF");
              client.println("<br>");
              client.print("Temperature: ");
              client.println(myMHZ19.getTemperature(true, true));  // force to get the value in float
              client.println("<br>");
              client.print("CO2Raw: ");
              client.println(myMHZ19.getCO2Raw(true));   // force to get the value in float
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
            client.println("------");
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

          if (co2_sensor == MHZ14A) {
            Calibrate_MHZ14A();
          }

          if (co2_sensor == SCD30) {
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

    co2_sensor = SCD30;

    // display device info
    SCD30DeviceInfo();

    // Set SCD30 (change in global variables at the start of SCD section in the initial section of this program)
    SCD30_Do_Measurement_Interval();
    SCD30_Do_Temperature_Offset();
    SCD30_Do_Altitude_Compensation();
    SCD30_Do_AutoSelfCalibration();

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
      display.setTextAlignment(TEXT_ALIGN_LEFT);
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

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Anaire30ppm");
    display.drawString(0, 10, "ID " + String(anaire_device_id));
    display.drawString(0, 20, String(counter));
    display.display(); // update OLED display

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
      myMHZ19.autoCalibration(eepromConfig.ABC);     // make sure auto calibration is off
      //Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status

      // Set range to 5000
      //myMHZ19.setRange(5000);
      myMHZ19.setRange(2000);

      char myVersion[4];
      myMHZ19.getVersion(myVersion);

      Serial.print("\nFirmware Version: ");
      for (byte i = 0; i < 4; i++)
      {
        Serial.print(myVersion[i]);
        if (i == 1)
          Serial.print(".");
      }
      Serial.println();
      Serial.print("Range: ");
      Serial.println(myMHZ19.getRange());
      Serial.print("Background CO2: ");
      Serial.println(myMHZ19.getBackgroundCO2());
      Serial.print("Temperature Cal: ");
      Serial.println(myMHZ19.getTempAdjustment());
      Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");

      // Filter incorrect values
      myMHZ19.setFilter(true, true);

      err_co2 = false;
      co2_sensor = MHZ14A;

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
        display.setTextAlignment(TEXT_ALIGN_LEFT);
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

      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 0, "Anaire50ppm");
      display.drawString(0, 10, "ID " + String(anaire_device_id));
      display.drawString(0, 20, String(counter));
      display.display(); // update OLED display

      // Print info
      Serial.println ("Warming up MHZ14A CO2 sensor complete");

      // Print info
      Serial.println ("MHZ14A CO2 sensor setup complete");

      // Setup DHT sensor
      dht.setup(DHT_GPIO, DHTesp::AUTO_DETECT);

      Serial.println ("DHT sensor setup complete, model detected (1=DHT11, 2=DHT22): ");
      Serial.println (dht.getModel());
    }

  }

}

// Read sensors
void Read_Sensors() {

  switch (co2_sensor) {
    case SCD30:
      Read_SCD30(); // Read co2, temperature and humidity
      break;
    case MHZ14A:
      Read_MHZ14A(); // Read co2 and temperature
      Read_DHT();  // Read temperature and humidity
      break;
    case none:
      break;
  }
}

// Read MHZ14A CO2 sensor
void Read_MHZ14A() {

  // Timestamp for serial up start time
  //int serial_up_start = millis();

  // clears out any garbage in the RX buffer
  //while (((mySerial.available()) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
  //  int garbage = mySerial.read();
  //  delay(100);
  //  Serial.println ("Cleaning mySerial data...");
  //}

  // Send out read command to the sensor - 9 bytes
  //mySerial.write(measurement_command, 9);

  // pauses the sketch and waits for the TX buffer to send all its data to the sensor
  //mySerial.flush();

  // Timestamp for serial up start time
  //serial_up_start = millis();

  // pauses the sketch and waits for the sensor response
  //if (((!mySerial.available()) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
  //  Serial.println ("Waiting for mySerial data...");
  //  delay(1000);
  //}

  // once data is available, it reads it to a variable 9 bytes
  //mySerial.readBytes(response_CO2, 9);

  // calculates CO2ppm value
  //response_CO2_high = (int)response_CO2[2];
  //response_CO2_low = (int)response_CO2[3];
  //CO2ppm_value = (256 * response_CO2_high) + response_CO2_low;

  // WiWaf library
  CO2ppm_value = myMHZ19.getCO2(true);

  if (myMHZ19.errorCode != RESULT_OK)             // RESULT_OK is an alias for 1. Either can be used to confirm the response was OK.
  {
    //err_co2 = true;
    Serial.println("Failed to receive CO2 value - Error");
    Serial.print("Response Code: ");
    Serial.println(myMHZ19.errorCode);          // Get the Error Code value
  }

  // prints calculated CO2ppm value to serial monitor
  Serial.print ("MHZ14A/19c CO2ppm_value: ");
  Serial.println (CO2ppm_value);

  // WiWaf library
  //temperature = (float) myMHZ19.getTemperature(true, true)/10;
  //if(myMHZ19.errorCode != RESULT_OK)              // RESULT_OK is an alias for 1. Either can be used to confirm the response was OK.
  //{
  //  Serial.println("Failed to receive temperature value - Error");
  //  Serial.print("Response Code: ");
  //  Serial.println(myMHZ19.errorCode);          // Get the Error Code value
  //}

  // prints temperature value to serial monitor
  //Serial.print ("MHZ14A/19c temperature: ");
  //Serial.println (temperature);

}

// Calibrate MHZ14A sensor
// Requires that the device is localed on open air for several minutes
void Calibrate_MHZ14A() {

  // Print info
  Serial.println ("Calibrating Winsen MHZ14A/MHZ19x CO2 sensor...");

  // Device status LED ON
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  //display.init();
  //display.flipScreenVertically();

  // Wait for calibrating time to stabalise...
  int counter = MHZ14A_CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < MHZ14A_CALIBRATION_TIME) {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Calibrando");
    display.drawString(0, 16, "MHZ14A " + String(counter));
    display.display(); // update OLED display
    // if not there are not connectivity errors, receive MQTT messages, to be able to interrupt calibration process
    if ((!err_MQTT) && (!err_wifi)) {
      MQTT_client.loop();
    }
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

  // Device status LED OFF
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);

}

// Read Sensirion SCD30 CO2, humidity and temperature sensor
void Read_SCD30()
{

  // Init I2C bus for SCD30
  //SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO, false);

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

  // Device status LED ON
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  //display.init();
  //display.flipScreenVertically();

  // Wait for calibrating time while reading values at maximum speed
  int counter = SCD30_CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < SCD30_CALIBRATION_TIME) {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Calibrando");
    display.drawString(0, 16, "SCD30 " + String(counter));
    display.display(); // update OLED display
    // if not there are not connectivity errors, receive MQTT messages, to be able to interrupt calibration process
    if ((!err_MQTT) && (!err_wifi)) {
      MQTT_client.loop();
    }
    Serial.print(".");
    delay(500);
    Serial.println(".");
    delay(500);
    counter = counter - 1;
  }

  // Send forced calibration command setting as zero reference value
  SCD30_Do_Forced_Calibration_Factor();

  delay(1000);

  // Print info
  Serial.println ("SCD30 sensor calibrated");

  // Device status LED OFF
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);

}

void SCD30_Do_AutoSelfCalibration()
{
  Serial.print("\nReading SCD30 AutoSelfCalibration before change: ");
  Serial.println(airSensor.getAutoSelfCalibration());
  Serial.print("Setting new SCD30 AutoSelfCalibration to: ");
  Serial.println(eepromConfig.ABC);

  if (airSensor.setAutoSelfCalibration(eepromConfig.ABC)) {
    Serial.print("Reading SCD30 AutoSelfCalibration after change: ");
    Serial.println(airSensor.getAutoSelfCalibration());
  }
  else {
    Serial.println("Could not set new SCD30 AutoSelfCalibration");
  }
}

void SCD30_Do_Temperature_Offset()
{
  uint16_t val;

  if (airSensor.getTemperatureOffset(&val)) {

    Serial.print("\nReading SCD30 Temperature Offset before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Temperature Offset to: ");
    Serial.println(eepromConfig.temperature_offset);

    if ( airSensor.setTemperatureOffset(eepromConfig.temperature_offset) ) {

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

}

void SCD30_Do_Measurement_Interval()
{
  uint16_t val;

  if (airSensor.getMeasurementInterval(&val)) {
    Serial.print("\nReading SCD30 Measurement Interval before change: ");
    Serial.println(val);

    Serial.print("Setting SCD30 new Measurement Interval to: ");
    Serial.println(SCD30_MEASUREMENT_INTERVAL);

    if (airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL)) {
      Serial.print("Change SCD30 Measurement Interval to: ");
      Serial.println(SCD30_MEASUREMENT_INTERVAL);

      if ( airSensor.getMeasurementInterval(&val) ) {
        Serial.print("Reading SCD30 Measurement Interval after change: ");
        Serial.println(val);
      }
      else {
        Serial.println("Could not obtain SCD30 Measurement Interval");
      }

    }
    else {
      Serial.print("Could not change SCD30 Measurement Interval to ");
      Serial.println(SCD30_MEASUREMENT_INTERVAL);
    }

  }
  else {
    Serial.println("Could not obtain SCD30 Measurement Interval");
  }

}

void SCD30_Do_Forced_Calibration_Factor()
{
  uint16_t val;
  bool result;

  if (airSensor.getForceRecalibration(&val) ) {
    Serial.print("\nReading SCD30 Forced Calibration Factor before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Forced Calibration Factor to: ");
    Serial.println(eepromConfig.FRC_value);

    result = airSensor.setForceRecalibration(eepromConfig.FRC_value);
    Serial.print("\nResponse after sending FRC command: ");
    Serial.println(result);
    delay (1000);

    if (airSensor.getForceRecalibration(&val)) {
      Serial.print("Reading SCD30 Forced Calibration Factor after change: ");
      Serial.println(val);
    }
    else {
      Serial.println("Could not obtain SCD30 forced calibration factor");
    }

  }
  else {
    Serial.println("Could not obtain SCD30 Forced Calibration Factor");
  }

}

void SCD30_Do_Altitude_Compensation()
{

  /* paulvha : you can set EITHER the Altitude compensation of the pressure.
     Setting both does not make sense as both overrule each other, but it is included for demonstration

     see Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf

        The CO2 measurement value can be compensated for ambient pressure by feeding the pressure value in mBar to the sensor.
        Setting the ambient pressure will overwrite previous and future settings of altitude compensation. Setting the argument to zero
        will deactivate the ambient pressure compensation. For setting a new ambient pressure when continuous measurement is running
        the whole command has to be written to SCD30.

        Setting altitude is disregarded when an ambient pressure is given to the sensor
  */

  uint16_t val;

  if ( airSensor.getAltitudeCompensation(&val) ) {
    Serial.print("\nReading SCD30 Altitude Compensation before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Altitude Compensation to: ");
    Serial.println(eepromConfig.altitude_compensation);

    if (airSensor.setAltitudeCompensation(eepromConfig.altitude_compensation)) {

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

}

void SCD30DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) + 1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 33 digits (32 serial + 0x0)

  if (airSensor.getSerialNumber(buf))
  {
    Serial.print(F("SCD30 serial number: "));
    Serial.println(buf);
  }

  // read Firmware level
  if ( airSensor.getFirmwareLevel(val) ) {
    Serial.print("SCD30 Firmware level - Major: ");
    Serial.print(val[0]);

    Serial.print("\t, Minor: ");
    Serial.println(val[1]);
  }
  else {
    Serial.println("Could not obtain firmware level");
  }

}

// Evaluate CO2 value versus warning and alarm threasholds and process CO2 alarm information
void Evaluate_CO2_Value() {

  // Status: ok

  // Recovering to "ok" status stops any warning or alarm warnings and quits special mode (after pressing flash button), therefore after passing by "ok" status
  // the device is "reset" and when entering warning or alarm state the device will report localy again by blinking o2_builtin_led_gpio16 led and the buzzer,
  // and pushing the flash button will be required if the user wanrts to stop lignt and sound alerting

  if (CO2ppm_value < eepromConfig.CO2ppm_warning_threshold) {

    co2_device_status = ok;                             // update co2 status
    alarm_ack = false;                                  // Init alarm ack status

    blinker_CO2_STATUS_BUILTIN_LED_GPIO.detach();       // stop CO2_STATUS_BUILTIN_LED_GPIO blinking
    digitalWrite(CO2_STATUS_BUILTIN_LED_GPIO, HIGH);    // update CO2_STATUS_BUILTIN_LED_GPIO_gpio1 to OFF

    blinker_BUZZER_GPIO.detach();                       // stop buzzer blinking
    digitalWrite(BUZZER_GPIO, LOW);                     // update BUZZER_GPIO to LOW to stop sound

  }

  // Status: warning
  else if ((CO2ppm_value >= eepromConfig.CO2ppm_warning_threshold) && (CO2ppm_value < eepromConfig.CO2ppm_alarm_threshold)) {

    co2_device_status = warning;                  // update device status

    blinker_CO2_STATUS_BUILTIN_LED_GPIO.attach_ms(WARNING_BLINK_PERIOD, changeState_CO2_STATUS_BUILTIN_LED_GPIO); // warning blink of light on CO2 LED

    if ((!alarm_ack) && (eepromConfig.sound_alarm)) {                                   // flash button hasn't been pressed to disable and sound_alarm is enabled
      blinker_BUZZER_GPIO.attach_ms(WARNING_BLINK_PERIOD, changeState_BUZZER_GPIO);     // warning sound on buzzer
    }

  }

  // Status: alarm
  else {

    co2_device_status = alarm; // update device status

    blinker_CO2_STATUS_BUILTIN_LED_GPIO.attach_ms(ALARM_BLINK_PERIOD, changeState_CO2_STATUS_BUILTIN_LED_GPIO);   // warning blink of light on CO2 LED

    if ((!alarm_ack) && (eepromConfig.sound_alarm)) {                                   // flash button hasn't been pressed to disable and sound_alarm is enabled
      blinker_BUZZER_GPIO.attach_ms(ALARM_BLINK_PERIOD, changeState_BUZZER_GPIO);       // alarm sound on buzzer
    }

  }

  // Print info on serial monitor
  switch (co2_device_status) {
    case ok:
      Serial.println ("STATUS: CO2 OK");
      break;
    case warning:
      Serial.println ("STATUS: CO2 WARNING");
      break;
    case alarm:
      Serial.println ("STATUS: CO2 ALARM");
      break;
  }

  // Update display OLED with new values
  update_OLED_co2_flag = true;

}

// Read temperature and humidity values from DHT sensor
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
void Read_DHT() {

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
    Serial.print("DHT Humidity: ");
    Serial.print(humidity);
    Serial.print(" % \n");
    Serial.print("DHT Temperature: ");
    Serial.print(temperature);
    Serial.println("ºC");
  }

}

// Send measurements to the cloud application by MQTT
void Send_Message_Cloud_App_MQTT() {

  // Print info
  Serial.print("Sending MQTT message to the send topic: ");
  Serial.println(MQTT_send_topic);
  sprintf(MQTT_message, "{id: %s,CO2: %d,humidity: %f,temperature: %f}", anaire_device_id.c_str(), (int) (CO2ppm_accumulated / CO2ppm_samples), humidity, temperature);
  Serial.print(MQTT_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents
  MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);

}

// callback function to receive configuration messages from the cloud application by MQTT
void Receive_Message_Cloud_App_MQTT(char* topic, byte* payload, unsigned int length) {

  //StaticJsonDocument<300> jsonBuffer;
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
    write_eeprom = true;
    blinker_BUZZER_GPIO.detach();                       // stop buzzer blinking
    digitalWrite(BUZZER_GPIO, LOW);                     // update BUZZER_GPIO to LOW to stop sound
    Serial.println("Alarm sound value: OFF");
  }

  if ((jsonBuffer["alarm"]) && ((!eepromConfig.sound_alarm) && (jsonBuffer["alarm"] == "ON"))) {
    eepromConfig.sound_alarm = true;
    Evaluate_CO2_Value();
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
    if (co2_sensor == MHZ14A) {
      //MHZ14A_Do_Temperature_Offset();
    }
    if (co2_sensor == SCD30) {
      SCD30_Do_Temperature_Offset();
    }
  }

  // Check altitude_compensation
  if ((jsonBuffer["altitude_compensation"]) && (eepromConfig.altitude_compensation != (uint16_t)jsonBuffer["altitude_compensation"])) {
    eepromConfig.altitude_compensation = (uint16_t)jsonBuffer["altitude_compensation"];
    if (co2_sensor == MHZ14A) {
      //MHZ14A_Do_Altitude_Compensation();
    }
    if (co2_sensor == SCD30) {
      SCD30_Do_Altitude_Compensation();
    }
  }

  // If calibration has been enabled, justo do it
  if ((jsonBuffer["FRC"]) && (jsonBuffer["FRC"] == "ON")) {
    if (co2_sensor == MHZ14A) {
      Calibrate_MHZ14A();
    }
    if (co2_sensor == SCD30) {
      Calibrate_SCD30();
    }
  }

  // Update ABC Automatic Baseline Calibration
  if ((jsonBuffer["ABC"]) && ((eepromConfig.ABC) && (jsonBuffer["ABC"] == "OFF"))) {
    eepromConfig.ABC = false;
    write_eeprom = true;
    if (co2_sensor == MHZ14A) {
      //MHZ14A_Do_AutoSelfCalibration()
    }
    if (co2_sensor == SCD30) {
      SCD30_Do_AutoSelfCalibration();
    }
    Serial.println("ABC: OFF");
  }

  if ((jsonBuffer["ABC"]) && ((!eepromConfig.ABC) && (jsonBuffer["ABC"] == "ON"))) {
    eepromConfig.ABC = true;
    write_eeprom = true;
    if (co2_sensor == MHZ14A) {
      //MHZ14A_Do_AutoSelfCalibration()
    }
    if (co2_sensor == SCD30) {
      SCD30_Do_AutoSelfCalibration();
    }
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
    firmware_update();
  }

  // If factory reset has been enabled, just do it
  if ((jsonBuffer["factory_reset"]) && (jsonBuffer["factory_reset"] == "ON")) {
    Wipe_EEPROM ();   // Wipe EEPROM
    ESP.reset();      // This is a bit crude. For some unknown reason webserver can only be started once per boot up
  }

  // If reboot, just do it, without cleaning the EEPROM
  if ((jsonBuffer["reboot"]) && (jsonBuffer["reboot"] == "ON")) {
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up
  }

  //print info
  Serial.println("MQTT update - message processed");
  Print_Config();

  // save the new values
  if (write_eeprom) {
    Write_EEPROM();
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

// ISR to respond to button pressing to toggle local alarm reporting through builtin LED and buzzer
// Enters each time the button is pressed
ICACHE_RAM_ATTR void push_button_handler() {

  flash_button_press_start = millis();        // Save press button start time to keep a defined time the device info on screen, and launch calibration if after 3 seconds the button si still pressed (LOW)
  flash_button_pressed_flag = true;

  //Serial.print ("FLASH Push button start: ");
  //Serial.println (flash_button_press_start);

  if (!alarm_ack) {
    Serial.println ("FLASH Push button interrupt - alarm_ack ON");
    // Switch off the buzzer to stop the sound alarm
    blinker_BUZZER_GPIO.detach();
    digitalWrite(BUZZER_GPIO, LOW);
    alarm_ack = true; // alarm has been ack
  }

  else {
    Serial.println ("FLASH Push button interrupt - alarm_ack OFF");
    alarm_ack = false; // alarm has been reset
    // Evaluate last CO2 measurement and update buzzer accordingly
    Evaluate_CO2_Value();
  }

  // Set flag to print device info in OLED display from the control loop
  update_OLED_status_flag = true;

}

// To blink on CO2_STATUS_BUILTIN_LED_GPIO
void changeState_CO2_STATUS_BUILTIN_LED_GPIO() {
  digitalWrite(CO2_STATUS_BUILTIN_LED_GPIO, !(digitalRead(CO2_STATUS_BUILTIN_LED_GPIO)));         //Invert Current State of LED CO2_STATUS_BUILTIN_LED_GPIO
  update_OLED_co2_flag = true;
}

// To blink on BUZZER_GPIO
void changeState_BUZZER_GPIO() {
  digitalWrite(BUZZER_GPIO, !(digitalRead(BUZZER_GPIO)));                                         //Invert Current State of BUZZER
}

// Update CO2 info on OLED display
void update_OLED_CO2() {

  // setup display and text format
  display.init();
  display.flipScreenVertically();
  display.clear();

  // display CO2 measurement
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  //display.setFont(ArialMT_Plain_24);
  //display.drawString(0, 4, String(CO2ppm_value) + "ppm");
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 8, "ppm");
  display.setFont(ArialMT_Plain_24);
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  //display.drawString(36, 4, String(CO2ppm_value));
  display.drawString(64, 4, String(CO2ppm_value));
  
  // And temperature and humidity
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(128, 0, String(int(temperature)) + "º  ");
  display.drawString(128, 16, String(int(humidity)) + "%");
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
  if (err_co2) {
    display.drawString(0, 20, "ERR CO2");
  }
  else if (err_wifi) {
    display.drawString(0, 20, "ERR WIFI");
  }
  else if (err_MQTT) {
    display.drawString(0, 20, "ERR MQTT");
  }
  else if (err_dht) {
    display.drawString(0, 20, "ERR DHT");
  }
  else {
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

  // Wipe EEPROM
  /*
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
    Print_Config();

  } else {
    anaire_device_id.toCharArray(eepromConfig.anaire_device_name, sizeof(eepromConfig.anaire_device_name));
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

void Wipe_EEPROM () {
  boolean result = EEPROM.wipe();
  if (result) {
    Serial.println("All EEPROM data wiped");
  } else {
    Serial.println("EEPROM data could not be wiped from flash store");
  }
}

void Print_Config() {
  Serial.println("#######################################");
  Serial.print("device id: ");
  Serial.println(anaire_device_id);
  Serial.print("anaire device name: ");
  Serial.println(eepromConfig.anaire_device_name);
  Serial.print("SW version: ");
  Serial.println(sw_version);
  Serial.print("WiFi SSID: ");
  Serial.println(wifi_ssid);
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

void firmware_update() {

  // For remote firmware update
  BearSSL::WiFiClientSecure UpdateClient;
  int freeheap = ESP.getFreeHeap();
  
  Serial.println("### FIRMWARE UPGRADE ###");

  // Add optional callback notifiers
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);
  UpdateClient.setInsecure();

  // Try to set a smaller buffer size for BearSSL update
  bool mfln = UpdateClient.probeMaxFragmentLength("raw.githubusercontent.com", 443, 512);
  Serial.printf("\nConnecting to https://raw.githubusercontent.com\n");
  Serial.printf("MFLN supported: %s\n", mfln ? "yes" : "no");
  if (mfln) {
    UpdateClient.setBufferSizes(512, 512);
  }
  UpdateClient.connect("raw.githubusercontent.com", 443);
  if (UpdateClient.connected()) {
    Serial.printf("MFLN status: %s\n", UpdateClient.getMFLNStatus() ? "true" : "false");
    Serial.printf("Memory used: %d\n", freeheap - ESP.getFreeHeap());
    freeheap -= ESP.getFreeHeap();
  } else {
    Serial.printf("Unable to connect\n");
  }

  // Run http update
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin");

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
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
  updating = true;
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
  Serial.println("### FIRMWARE UPGRADE COMPLETED - REBOOT ###");
  updating = false;
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
  updating = false;
}
