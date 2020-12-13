// From 20201109 ANAIRE DEVICE CODE - anaire@anaire.org
// GNU General Public License v3.0
//
// Get CO2, temperature and humidity measurements and send them to the anaire cloud app
//
// Parts:
// AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
// MH-Z14A - CO2 sensor. Connected by serial port (swSerial on NodeMCU) http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf
// AZ-Delivery DHT11 Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
// AZ-Delivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
// AZ-Delivery 0.91 inch OLED I2C Display 128 x 32 Pixels  https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display
//
// Parts list for a complete prototype in amazon: https://www.amazon.es/hz/wishlist/ls/8NAKLGML187W?ref_=wl_share
//
// Arduino IDE Setup:
// Install the usb to uart driver from https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
// Start Arduino and open Preferences window.
// Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
// Open Boards Manager from Tools > Board menu and find esp8266 platform by esp8266 community and install the software for Arduino from a drop-down box.
// Select "NodeMCU 1.0" board from Tools > Board menu after installation
//
// Install the following libraries in Arduino IDE:
// WiFiEsp for WiFi https://github.com/bportaluri/WiFiEsp
// DHTesp - to manage DHT11 or DHT22 temperature and humidity sensors https://github.com/beegee-tokyo/DHTesp
// EspSoftwareSerial - to manage sw serial port to communicate with CO2 sensor https://github.com/plerup/espsoftwareserial/
// ArduinoMqttClient - for MQTT communications https://github.com/arduino-libraries/ArduinoMqttClient
// esp8266-oled-ssd1306 for oled display https://github.com/ThingPulse/esp8266-oled-ssd1306
// SparkFun_SCD30_Arduino_Library - for Sensirion SCD30 CO2, humidity and temperature sensor https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library

// Design:
// - Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector) is also connected to the external buzzer
//   * When CO2 Status is "ok" (below warning threshold) LED keeps ON and buzzer is off
//   * When CO2 Status is "warning" builtin LED and external buzzer alternate at a slow pace (WARNING_BLINK_PERIOD)
//   * When CO2 Status is "alarm" builtin LED and external buzzer alternate at fast pace (ALARM_BLINK_PERIOD)
//   This GPIO16 is also used when downloading SW to the nodemcu board: the external buzzer sounds while downloading to the board
// - Flash button, to the risht of the USB connector, allows to toggle between activating/deactivating local alarms (visual and sound)
//   * It also shows device ID and IP address after being pressed until a new CO2 measurement is displayed
// - CO2 sensor is connected through a software serial port using GPIO13(D7) and GPIO15 (D8)
// - DHT11 humidity and temperature sensor ins connected on GPIO 5 (D1)
// - SSD1306 OLED display is connected on GPIO14(D5) and GPIO12 (D6)
// - The device is designed to work only with the CO2 sensor, so buzzer, DHT11 humidity and temperature sensor, and OLED display are optional
// - The device is designed to recover from Wifi, MQTT or sensor reading temporal failures
// - The web server is activated, therefore entering the IP on a browser allows to see the device data. The IP is showed during boot and after pressing the button.

// edit the following file to configure the device: device id, wifi network and remote cloud endpoint
#include "anaire_config.h"

// Control Loop: time between measurements
const int CONTROL_LOOP_DURATION = 30000; // 30 seconds
unsigned long control_loop_start; // holds a timestamp for each control loop start

// CO2 Blinking period, used to reflect CO2 status on builtin led and buzzer
const int WARNING_BLINK_PERIOD = 1000; // 1 second
const int ALARM_BLINK_PERIOD = 100;    // 0.1 seconds

// WiFi
#include <ESP8266WiFi.h> // Wifi ESP8266
#include <ESP8266mDNS.h> // to be reached on anaire_device_id.local in the local network
WiFiClient wifi_client;
const int WIFI_CONNECT_TIMEOUT = 10000; // 10 seconds
int wifi_status = WL_IDLE_STATUS;
//String firmware_version;
//long wifi_rssi_dbm; // Received signal strength
//IPAddress ip_address; // Wifi shield IP address
WiFiServer wifi_server(80); // to check if it is alive
//char wifi_hostname[sizeof(anaire_device_id)];

// OLED ssd1306 screen
#include <Wire.h>
#include "SSD1306Wire.h"

// SCL and SDA pin connections
//#define OLED_SCK_GPIO 14 // signal GPIO14 (D5)
//#define OLED_SDA_GPIO 12 // signal GPIO12 (D6)
//#define OLED_SCK_GPIO 2 // signal GPIO2 (D4)
//#define OLED_SDA_GPIO 0 // signal GPIO0 (D3)
#define OLED_SCK_GPIO 4 // signal GPIO2 (D4)
#define OLED_SDA_GPIO 2 // signal GPIO0 (D3)
// for 128x32 displays:
SSD1306Wire display(0x3c, OLED_SDA_GPIO, OLED_SCK_GPIO, GEOMETRY_128_32);  // ADDRESS, SDA, SCL

// MQTT
#include <ArduinoMqttClient.h>
MqttClient mqttClient(wifi_client);
const char mqtt_send_topic[]  = "measurement";
const char mqtt_receive_topic[]  = "config";
char mqtt_message[256];

#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
SCD30 airSensor;

// MH-Z14A CO2 sensor: software serial port
const unsigned long CO2_WARMING_TIME = 18000;   // MH-Z14A CO2 sensor warming time: 3 minutes = 180000 ms
const unsigned long CO2_SERIAL_TIMEOUT = 5000;   // MH-Z14A CO2 serial start timeout: 5 seconds = 5000 ms
const unsigned long CALIBRATION_TIME = 1200000;  // MH-Z14A CO2 CALIBRATION TIME: 20 min = 1200000 ms
#include "SoftwareSerial.h"
#define swSerialRX_gpio 13
#define swSerialTX_gpio 15
SoftwareSerial swSerial(swSerialRX_gpio, swSerialTX_gpio, false);
byte measurement_command[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // Command to get measurements from MH-Z14A CO2 sensor
byte calibration_command[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // Command to calibrate MH-Z14A CO2 sensor
char response_CO2[9]; // holds the received data from MH-Z14A CO2 sensor
int response_CO2_high; // holds upper byte
int response_CO2_low;  // holds lower byte
int CO2ppm_value = 0;  // CO2 ppm measured value

// AZ-Delivery DHT11
#include "DHTesp.h"
//#define DHT_GPIO 5        // signal GPIO5 (D1)
#define DHTTYPE DHT11     // DHT (AM2302) Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//#define DHTTYPE DHT22   // DHT 22 (AM2302)

// Initialize DHT sensor
DHTesp dht;
float temperature;  // Read temperature as Celsius
float humidity;     // Read humidity in %

// AZ-Delivery Active Buzzer: GPIO16 (D0)
// This is used to leverage existing buitlin LED in nodemcu connected to GPIO16.
// int buzzer_gpio = 16;
// It is not required to configure it, as it is configured and managed later on the builtin led configuration

// GPIO 0 is button FLASH on nodemcu PCB
int push_button_gpio = 0; // Flash button

// Status info
// First builtin LED, used to provide CO2 visual status info, as the buzzer produces sound info on the same pin on nodemcu v1.0 board
int co2_builtin_LED = 16;     // GPIO16 (D0), closer to the usb port, on the NodeMCU PCB
// Second builtin LED on  used to provide device status info
int status_builtin_LED = 2;   // GPIO2 (D4), on the ESP-12 module’s PCB
int alarm_ack = false;        // to indicate if push button has been pushed to ack the alarm and switch off the buzzer

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
  pinMode(status_builtin_LED, OUTPUT);

  // Turn on both, to show init status
  digitalWrite(co2_builtin_LED, LOW);
  digitalWrite(status_builtin_LED, LOW);

  // Print welcome screen on OLED Display
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "anaire.org");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 22, "anaireslim device");
  display.display(); // update OLED display
  delay(3000);
  
  
  // Initialize MH-Z14A CO2 sensor: warming up and calibrate
  //Setup_MHZ14A();
  Setup_SCD30();

  /*
    // Initialize DHT11 temperature and humidity sensor
    dht.setup(DHT_GPIO, DHTesp::DHTTYPE);
    if (dht.getStatus() != 0) {
    err_dht = true;
    }
  */

  // Attempt to connect to WiFi network:
  Connect_WiFi();

  // Attempt to connect to MQTT broker
  if (!err_wifi) {
    Init_MQTT();
  }

  // Push button: attach interrupt to togle local report of CO2 status by blinking and buzzer
  pinMode(push_button_gpio, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(push_button_gpio), push_button_handler, FALLING);

  // turn off both, to show init status complete
  digitalWrite(co2_builtin_LED, HIGH);
  digitalWrite(status_builtin_LED, HIGH);

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  Serial.println ("--- LOOP BEGIN ---");

  // Timestamp for the loop start time
  control_loop_start = millis();

  // Turn on status LED to indicate the start of measurement and evaluation process. This acts as a heart beat to indicate the device is alive, by showing activity each control loop duration time
  digitalWrite(status_builtin_LED, LOW);

  // Read CO2 sensor
  //Read_MHZ14A();
  Read_SCD30();

  // Evaluate CO2 value
  Evaluate_CO2_Value();

  // Read DHT sensor
  //Read_Humidity_Temperature();

  // Update CO2 measurement on screen
  update_OLED_CO2();

  // Message the mqtt broker in the cloud app to send the measured values
  if (!err_wifi) {
    Send_Message_Cloud_App_MQTT();
  }

  // Turn off builtin status LED to indicate the end of measurement and evaluation process
  digitalWrite(status_builtin_LED, HIGH);

  // Complete time up to ControlLoopTimerDuration and blink fast builtin LED to show it
  while ((millis() - control_loop_start) < CONTROL_LOOP_DURATION)
  {
    // Process wifi server requests
    Check_WiFi_Server();
    delay(500);

    // if button was pressed flag to updated OLED was set
    if (update_OLED_Status_flag) {
      update_OLED_Status_flag = false;
      update_OLED_Status();
    }

    if (err_co2) {
      // Initialize MH-Z14A CO2 sensor: warming up and calibrate
      //Setup_MHZ14A();
      Setup_SCD30();
    }

    // Try to recover error conditions
    if (err_wifi) {
      // Attempt to connect to WiFi network:
      Connect_WiFi();
    }

    if ((err_mqtt) && (!err_wifi)) {
      // Attempt to connect to MQTT broker
      Init_MQTT();
    }

    // if not there are not connectivity errors, receive mqtt messages
    if ((!err_mqtt) && (!err_wifi)) {
      //Receive_Message_Cloud_App_MQTT();
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
  Serial.println(ssid); // print the network name (SSID);

  // connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

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
    blinker_status_builtin_LED.attach_ms(ALARM_BLINK_PERIOD, changeState_status_builtin_LED);

  }
  else {
    err_wifi = false;
    // Switch off blinking status_builtin_LED
   blinker_status_builtin_LED.detach();

    wifi_server.begin(); // start the web server on port 80
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Set mDNS to anaire_device_id.local
    if (!MDNS.begin(String(anaire_device_id))) {
      Serial.println("Error mDNS");
    }
    else {
      Serial.print("Try to connect to ");
      Serial.print(String(anaire_device_id));
      Serial.println(".local");
    }

  }

  // Print ID & IP info on OLED display for 5 seconds
  //update_OLED_Status();
  //delay(10000);

}

void Init_MQTT() {
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.print(cloud_server_address);
  Serial.print(":");
  Serial.println(cloud_app_port);

  if (!mqttClient.connect(cloud_server_address, cloud_app_port)) {
    err_mqtt = true;
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    // Switch on blinking status_builtin_LED to reflect the error, with ALARM_BLINK_PERIOD
    blinker_status_builtin_LED.attach_ms(ALARM_BLINK_PERIOD, changeState_status_builtin_LED);
  }
  else {
    err_mqtt = false;
    Serial.println("You're connected to the MQTT broker!");

    // Switch off blinking status_builtin_LED
    blinker_status_builtin_LED.detach();

    // Set client id
    //mqttClient.setId(anaire_device_id);

    // subscribe to config topic, to receive configuration messages, with qos=1
    //mqttClient.subscribe(mqtt_receive_topic, 1);

    // Set mqtt clean session
    //mqttClient.setCleanSession(false);

    //Serial.print("Waiting for messages on topic: ");
    //Serial.println(mqtt_receive_topic);
  }

  // Print ID & IP info on OLED display for 5 seconds
  //update_OLED_Status();
  //delay(10000);

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
            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to calibrate the device<br>");
            //client.print("Click <a href=\"/L\">here</a> turn the LED on pin 6 off<br>");
            // Print current info
            client.print("Anaire Device ID: ");
            client.print(anaire_device_id);
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
            client.print("STATUS: ");
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
          //digitalWrite(LED, HIGH);               // GET /H turns the LED on
          Calibrate_MHZ14A();
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


// Setup MH-Z14A CO2 sensor
void Setup_MHZ14A()
{

  // Initialize serial port to communicate with MH-Z14A CO2 sensor. This is a software serial port
  swSerial.begin(9600, SWSERIAL_8N1, swSerialRX_gpio, swSerialTX_gpio, false, 128);

  Serial.println("swSerial Txd is on pin: " + String(swSerialTX_gpio));
  Serial.println("swSerial Rxd is on pin: " + String(swSerialRX_gpio));

  // Timestamp for serial up start time
  int serial_up_start = millis();

  while (((!swSerial) && ((millis() - serial_up_start) < CO2_SERIAL_TIMEOUT))) {
    Serial.println("Attempting to open serial port 1 to communicate to MH-Z14A CO2 sensor");
    delay(1000); // wait 1 seconds for connection
  }

  // If the timeout was completed it is an error
  if ((millis() - serial_up_start) > CO2_SERIAL_TIMEOUT) {
    err_co2 = true;
    return;
  }
  else {
    err_co2 = false;
  }

  // Timestamp for warming up start time
  int warming_up_start = millis();

  // Print info
  Serial.println ("Warming up MH-Z14A CO2 sensor...");

  // Wait for warming time while blinking blue led
  int counter = 180; // default warming up 180s
  while ((millis() - warming_up_start) < CO2_WARMING_TIME) {
    // Print welcome screen
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Init anaire.org");
    display.drawString(0, 10, String(anaire_device_id));
    display.drawString(0, 20, String(counter));
    display.display(); // update OLED display
    Serial.print(".");
    delay(500); // wait 500ms
    Serial.println(".");
    delay(500); // wait 500ms
    counter = counter - 1;
  }

  // Print info
  Serial.println ("Warming up MH-Z14A CO2 sensor complete");

  // Print info
  Serial.println ("MH-Z14A CO2 sensor setup complete");
}

// Read MH-Z14A CO2 sensor
void Read_MHZ14A()
{

  // Timestamp for serial up start time
  int serial_up_start = millis();

  // clears out any garbage in the RX buffer
  while (((swSerial.available()) && ((millis() - serial_up_start) < CO2_SERIAL_TIMEOUT))) {
    int garbage = swSerial.read();
    delay(100);
    Serial.println ("Cleaning swSerial data...");
  }

  // Send out read command to the sensor - 9 bytes
  swSerial.write(measurement_command, 9);

  // pauses the sketch and waits for the TX buffer to send all its data to the sensor
  swSerial.flush();

  // Timestamp for serial up start time
  serial_up_start = millis();

  // pauses the sketch and waits for the sensor response
  if (((!swSerial.available()) && ((millis() - serial_up_start) < CO2_SERIAL_TIMEOUT))) {
    Serial.println ("Waiting for swSerial data...");
    delay(1000);
  }

  // once data is available, it reads it to a variable 9 bytes
  swSerial.readBytes(response_CO2, 9);

  // calculates CO2ppm value
  response_CO2_high = (int)response_CO2[2];
  response_CO2_low = (int)response_CO2[3];
  CO2ppm_value = (256 * response_CO2_high) + response_CO2_low;

  // prints calculated CO2ppm value to serial monitor
  Serial.print ("CO2ppm_value = ");
  Serial.println (CO2ppm_value);

}

// Calibrate MHZ14A sensor
void Calibrate_MHZ14A() {

  // Print info
  Serial.println ("Calibrating MH-Z14A CO2 sensor...");

  // Write calibration command
  swSerial.write(calibration_command, 9);

  // Waits for 3 seconds for the command to take effect
  delay(3000);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  // Wait for calibrating time
  int counter = CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < CALIBRATION_TIME) {
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, "anaire.org");
    display.drawString(0, 21, "calibrating");
    display.drawString(0, 42, String(counter));
    display.display(); // update OLED display
    Serial.print(".");
    delay(500); // wait 500ms
    Serial.println(".");
    delay(500); // wait 500ms
    counter = counter - 1;
  }

}

// Setup Sensirion SCD30 CO2, humidity and temperature sensor
void Setup_SCD30()
{

  //Serial.begin(115200);
  Serial.println                                                                                                                                                                                                                                                                                  ("Setup SCD30 air sensor");
  
  //Wire.begin(4,5);
  //Wire.begin(7,6);
  Wire.begin(12,14);
  
  if (airSensor.begin(Wire) == false)
 
  {
    Serial.println("Air sensor not detected. Please check wiring.");
    //while (1);
    err_co2 = true;
    return;
  }

  Serial.println("Air sensor SCD-30 detected.");
  
  airSensor.setMeasurementInterval(15); //Change number of seconds between measurements: 2 to 1800 (30 minutes)

  //My desk is ~600m above sealevel
  airSensor.setAltitudeCompensation(650); // Madrid, barrio del Pilar

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  airSensor.setAmbientPressure(1000); //Current ambient pressure in mBar: 700 to 1200

  float offset = airSensor.getTemperatureOffset();
  Serial.print("Current temp offset: ");
  Serial.print(offset, 2);
  Serial.println("C");

  // Print info
  Serial.println ("Sensirion SCD-30 sensor setup complete");
 
}

// Read Sensirion SCD30 CO2, humidity and temperature sensor
void Read_SCD30()
{

  // Timestamp for serial up start time
  //int serial_up_start = millis();

  //Wire.begin(4,5);
  //Wire.begin(7,6);
  Wire.begin(12,14);

  if (airSensor.dataAvailable())
  {
    CO2ppm_value = airSensor.getCO2();
    temperature = airSensor.getTemperature();
    humidity = airSensor.getHumidity();
    
    Serial.print("co2(ppm):");
    Serial.print(CO2ppm_value);

    Serial.print(" temp(C):");
    Serial.print(temperature, 1);

    Serial.print(" humidity(%):");
    Serial.print(humidity, 1);

    Serial.println();
  }

}

// Calibrate Sensirion SCD30 CO2, humidity and temperature sensor
void Calibrate_SCD30() {

  // Print info
  Serial.println ("Calibrating SCD30 sensor...");

  // Write calibration command
  swSerial.write(calibration_command, 9);

  // Waits for 3 seconds for the command to take effect
  delay(3000);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  // Wait for calibrating time
  int counter = CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < CALIBRATION_TIME) {
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, "anaire.org");
    display.drawString(0, 21, "calibrating");
    display.drawString(0, 42, String(counter));
    display.display(); // update OLED display
    Serial.print(".");
    delay(500); // wait 500ms
    Serial.println(".");
    delay(500); // wait 500ms
    counter = counter - 1;
    
  }

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
    blinker_status_builtin_LED.detach(); // stop blinkg status_builtin_LED_gpio16 to indicate reset to init state
    alarm_ack = false; // Init alarm ack status
  }

  // Status: warning
  else if ((CO2ppm_value >= CO2ppm_warning_threshold) && (CO2ppm_value < CO2ppm_alarm_threshold)) {
    co2_device_status = warning; // update device status
    if (!alarm_ack) { // flash button hasn't been pressed
      blinker_co2_builtin_LED.attach_ms(WARNING_BLINK_PERIOD, changeState_co2_builtin_LED); // warning blink of light and sound
    }
  }

  // Status: alarm
  else {
    co2_device_status = alarm; // update device status
    if (!alarm_ack) { // flash button hasn't been pressed
      blinker_co2_builtin_LED.attach_ms(ALARM_BLINK_PERIOD, changeState_co2_builtin_LED); // warning blink of light and sound
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

// Read temperature and humidity values
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
void Read_Humidity_Temperature() {

  TempAndHumidity lastValues = dht.getTempAndHumidity();

  // Read humidity as percentage
  //humidity = dht_client.readHumidity();
  humidity = lastValues.humidity;

  // Read temperature as Celsius
  //temperature = dht_client.readTemperature();
  temperature = lastValues.temperature;

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    err_dht = true;
  }
  else {
    err_dht = false;
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%\n");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("ºC");
  }

}

// Send measurements to the cloud application by MQTT
void Send_Message_Cloud_App_MQTT() {

  // Print info
  Serial.println("Making MQTT request");

  if (!mqttClient.connect(cloud_server_address, cloud_app_port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    err_mqtt = true;
    return;
  }
  else {
    err_mqtt = false;
  }

  Serial.print("Sending mqtt message to the send topic ");
  Serial.println(mqtt_send_topic);
  sprintf(mqtt_message, "{id: %s,CO2: %d,humidity: %f,temperature: %f}", anaire_device_id, CO2ppm_value, humidity, temperature);
  Serial.print(mqtt_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(mqtt_send_topic);
  mqttClient.print(mqtt_message);
  mqttClient.endMessage();

}

// receive configuration messages from the cloud application by MQTT
void Receive_Message_Cloud_App_MQTT() {

  // Print info
  Serial.println("Receiving MQTT message");

  if (!mqttClient.connect(cloud_server_address, cloud_app_port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    err_mqtt = true;
    return;
  }
  else {
    err_mqtt = false;

    // subscribe to config topic, to receive configuration messages, with qos=1
    mqttClient.subscribe(mqtt_receive_topic, 1);

    // Set mqtt clean session
    mqttClient.setCleanSession(false);

    int messageSize = mqttClient.parseMessage();

    if (messageSize) {
      // we received a message, print out the topic and contents
      Serial.print("Received a message with topic '");
      Serial.print(mqttClient.messageTopic());
      Serial.print("', length ");
      Serial.print(messageSize);
      Serial.println(" bytes:");

      // use the Stream interface to print the contents
      while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
      }
      Serial.println();
    }

  }

}

// ISR to respond to button pressing to toggle local alarm reporting thoriugh builtin LED and buzzer
ICACHE_RAM_ATTR void push_button_handler() {

  if (!alarm_ack) {

    // Print info
    Serial.println ("FLASH Push button interrupt - alarm_ack ON");

    // Switch off the buzzer and co2_builtin_LED, as in normal status
    blinker_co2_builtin_LED.detach(); //Use attach_ms if you need time in ms CONTROL_LOOP_DURATION
    digitalWrite(co2_builtin_LED, LOW);

    // Switch on blinking status_builtin_LED to reflect special mode, using WARNING_BLINK_PERIOD
    blinker_status_builtin_LED.attach_ms(WARNING_BLINK_PERIOD, changeState_status_builtin_LED);

    alarm_ack = true; // alarm has been ack

  }

  else {

    // Print info
    Serial.println ("FLASH Push button interrupt - alarm_ack OFF");

    // Switch off blinking status_builtin_LED to reflect normal mode
    blinker_status_builtin_LED.detach();

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
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  // display CO2 measurement on first line
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
        display.drawString(0, 16, "CO2 MAL: Ventile");
        break;
    }
  }

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
  display.drawString(0, 0, String(anaire_device_id));

  // display IP address on second line
  String ipaddress = WiFi.localIP().toString();
  display.drawStringMaxWidth(0, 10, 128, ipaddress);

  // if there is an error display it on third line
  if (err_wifi) {
    display.drawString(0, 20, "err wifi");
  }
  else if (err_mqtt) {
    display.drawString(0, 20, "err mqtt");
  }
  else if (err_co2) {
    display.drawString(0, 20, "err co2");
  }
  else if (err_dht) {
    display.drawString(0, 20, "err dht");
  }

  else {
    display.drawString(0, 20, "device ok");
  }

  display.display(); // update OLED display
  
}
