// From 20201109 ANAIRE DEVICE CODE - anaire@anaire.org
//
// Get CO2, temperature and humidity measurements and send them to the anaire cloud app
//
// AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
// MH-Z14A - CO2 sensor. Connected by serial port (swSerial on NodeMCU) http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z14%20CO2%20V2.4.pdf
// AZ-Delivery DHT11 Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
// AZ-Delivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
// AZ-Delivery 0.91 inch OLED I2C Display 128 x 32 Pixels  https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display
//
// Parts list for a complete prototype in amazon: https://www.amazon.es/hz/wishlist/ls/8NAKLGML187W?ref_=wl_share
//
// Arduino IDE Setup:
// Start Arduino and open Preferences window.
// Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
// Open Boards Manager from Tools > Board menu and find esp8266 platform by esp8266 community and install the software for Arduino from a drop-down box.
// Select "NodeMCU 1.0" board from Tools > Board menu after installation
//
// Install the following libraries in Arduino IDE:
// ESP8266WiFi for WiFi https://github.com/bportaluri/WiFiEsp
// DHTesp - to manage DHT11 or DHT22 temperature and humidity sensors https://github.com/markruys/arduino-DHT
// EspSoftwareSerial - to manage sw serial port to communicate with CO2 sensor https://github.com/plerup/espsoftwareserial/
// ArduinoMqttClient - for MQTT communications https://github.com/arduino-libraries/ArduinoMqttClient
// esp8266-oled-ssd1306 for oled display https://github.com/ThingPulse/esp8266-oled-ssd1306

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
#define OLED_SCL_GPIO 14 // signal GPIO14 (D5)
#define OLED_SDA_GPIO 12 // signal GPIO12 (D6)

// for 128x64 displays:
SSD1306Wire display(0x3c, OLED_SDA_GPIO, OLED_SCL_GPIO);  // ADDRESS, SDA, SCL

// MQTT
#include <ArduinoMqttClient.h>
MqttClient mqttClient(wifi_client);
const char mqtt_send_topic[]  = "measurement";
const char mqtt_receive_topic[]  = "config";
char mqtt_message[256];

// MH-Z14A CO2 sensor: software serial port
const unsigned long CO2_WARMING_TIME = 180000; // MH-Z14A CO2 sensor warming time: 3 minutes
const unsigned long CO2_SERIAL_TIMEOUT = 5000; // MH-Z14A CO2 serial start timeout: 5 seconds
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
#define DHT_GPIO 5        // signal GPIO5 (D1)
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
int co2_builtin_led_gpio16 = 16;    // GPIO16 (D0) builtin LED closer to the usb port, on the NodeMCU PCB, used to provide CO2 visual status info, as the buzzer produces sound info on the same pin
int status_builtin_led_gpio2 = 2;   // GPIO2 (D4) builtin LED on on the ESP-12 module’s PCB, used to provide device status info

int alarm_ack = false;                // to indicate if push button has been pushed to ack the alarm and switch off the buzzer

// CO2 device status
enum CO2_status {ok, warning, alarm}; // the device can have one of those CO2 status
CO2_status co2_device_status = ok;        // initialized to ok

// Global device status
enum Global_status {device_ok, err_wifi, err_mqtt, err_co2, err_th, err_oled, err_unknown}; // the device can have one of those Global status
Global_status global_device_status = device_ok;  // initialized to ok

#include <Ticker.h>  //Ticker Library
Ticker blinker;

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
  pinMode(co2_builtin_led_gpio16, OUTPUT);
  pinMode(status_builtin_led_gpio2, OUTPUT);

  // Turn on both, to show init status
  digitalWrite(co2_builtin_led_gpio16, LOW);
  digitalWrite(status_builtin_led_gpio2, LOW);

  // OLED Display init
  display.init();

  // Print welcome screen
  display.clear();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "anaire.org");
  display.drawString(0, 24, "INIT...");
  display.display();

  // Attempt to connect to WiFi network:
  Connect_WiFi();

  // Attempt to connect to MQTT broker
  Init_MQTT();

  // Buzzer: not required
  //pinMode(buzzer_gpio, OUTPUT);

  // Push button: attach interrupt to togle local report of CO2 status by blinking and buzzer
  pinMode(push_button_gpio, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(push_button_gpio), push_button_handler, FALLING);

  // Initialize MH-Z14A CO2 sensor: warming up and calibrate
  Setup_MHZ14A();

  // Initialize DHT11 temperature and humidity sensor
  dht.setup(DHT_GPIO, DHTesp::DHTTYPE);

  // turn off both, to show init status
  digitalWrite(co2_builtin_led_gpio16, HIGH);
  digitalWrite(status_builtin_led_gpio2, HIGH);

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  Serial.println ("--- LOOP BEGIN ---");

  // Timestamp for the loop start time
  control_loop_start = millis();

  // Turn on status LED to indicate the start of measurement and evaluation process. This acts as a heart beat to indicate the device is alive, by showing activity each control loop duration time
  digitalWrite(status_builtin_led_gpio2, LOW);

  // Read MH-Z14A CO2 sensor
  Read_MHZ14A();

  // Evaluate CO2 value
  Evaluate_CO2_Value();

  // Update CO2 measurement on screen
  update_OLED_CO2();

  // Read DHT sensor
  Read_Humidity_Temperature();

  // Message the mqtt broker in the cloud app to send the measured values
  Message_Cloud_App_MQTT();

  // Print status on serial port
  // Print_WiFi_Status();

  // Turn off builtin status LED to indicate the end of measurement and evaluation process
  digitalWrite(status_builtin_led_gpio2, HIGH);

  // Complete time up to ControlLoopTimerDuration and blink fast builtin LED to show it
  while ((millis() - control_loop_start) < CONTROL_LOOP_DURATION)
  {
    // Process wifi server requests
    Check_WiFi_Server();
    delay(1000);

    // Try to recover error conditions
    if (global_device_status == err_wifi) {
      // Attempt to connect to WiFi network:
      Connect_WiFi();
    }
    
    if (global_device_status == err_mqtt) {
      // Attempt to connect to MQTT broker
      Init_MQTT();
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
    global_device_status = err_wifi;
  }
  else {

    global_device_status = device_ok;
    
    wifi_server.begin(); // start the web server on port 80
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Print ID & IP info on OLED display for 5 seconds
    update_OLED_Status();
    delay(10000);
  
  }

}

void Init_MQTT() {
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.print(cloud_server_address);
  Serial.print(":");
  Serial.println(cloud_app_port);
  
  if (!mqttClient.connect(cloud_server_address, cloud_app_port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    global_device_status = err_mqtt;
  }
  else {
    Serial.println("You're connected to the MQTT broker!");
    global_device_status = device_ok;
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
            // the content of the HTTP response follows the header:
            //client.print("Click <a href=\"/H\">here</a> turn the LED on pin 6 on<br>");
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
  
  // Timestamp for warming up start time
  int warming_up_start = millis();

  // Print info
  Serial.println ("Warming up MH-Z14A CO2 sensor...");

  // Wait for warming time while blinking blue led
  while ((millis() - warming_up_start) < CO2_WARMING_TIME) {
    Serial.print(".");
    delay(500); // wait 500ms
    Serial.println(".");
    delay(500); // wait 500ms
  }

  // Print info
  Serial.println ("Warming up MH-Z14A CO2 sensor complete");

  // Print info
  //Serial.println ("Calibrating MH-Z14A CO2 sensor...");

  // Write calibration command
  //swSerial.write(calibration_command, 9);

  // Waits for 3 seconds
  //delay(3000);

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

// Evaluate CO2 value versus warning and alarm threasholds and process CO2 alarm information
void Evaluate_CO2_Value() {

  // Status: ok
  if (CO2ppm_value < CO2ppm_warning_threshold) {
    co2_device_status = ok; // update device status
    alarm_ack = false; // Init alarm ack status
    blinker.detach(); //Use attach_ms if you need time in ms CONTROL_LOOP_DURATION
    digitalWrite(co2_builtin_led_gpio16, LOW);
  }

  // Status: warning
  else if ((CO2ppm_value >= CO2ppm_warning_threshold) && (CO2ppm_value < CO2ppm_alarm_threshold)) {
    co2_device_status = warning; // update device status
    if (!alarm_ack) {
      blinker.attach_ms(WARNING_BLINK_PERIOD, changeState_co2_builtin_led_gpio16);
    }
  }

  // Status: alarm
  else {
    co2_device_status = alarm; // update device status
    if (!alarm_ack) {
      blinker.attach_ms(ALARM_BLINK_PERIOD, changeState_co2_builtin_led_gpio16);
    }
  }

  // Print info
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
  }

  else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%\n");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("ºC");
  }

}

// Send measurements to the cloud application by MQTT
void Message_Cloud_App_MQTT() {

  // Print info
  Serial.println("Making MQTT request");

  if (!mqttClient.connect(cloud_server_address, cloud_app_port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    //while (1);
  }

  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  //(mqttClient.poll();

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


ICACHE_RAM_ATTR void push_button_handler() {

  if (!alarm_ack) {

    // Print info
    Serial.println ("FLASH Push button interrupt - alarm_ack ON");

    // Switch off the buzzer and keep the LED on, as in normal status
    blinker.detach(); //Use attach_ms if you need time in ms CONTROL_LOOP_DURATION
    digitalWrite(co2_builtin_led_gpio16, LOW);

    alarm_ack = true; // alarm has been ack

  }

  else {

    // Print info
    Serial.println ("FLASH Push button interrupt - alarm_ack OFF");

    alarm_ack = false; // alarm has been reset

    // Evaluate last CO2 measurement and update led and buzzer accordingly
    Evaluate_CO2_Value();

  }

  // Print id and IP info in OLED display
  //update_OLED_Status();

}

// To blink on co2_builtin_led and buzzer
void changeState_co2_builtin_led_gpio16() {
  digitalWrite(co2_builtin_led_gpio16, !(digitalRead(co2_builtin_led_gpio16)));  //Invert Current State of LED co2_builtin_led_gpio16
}

// Update CO2 info on OLED display
void update_OLED_CO2() {

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);

  display.drawString(0, 0, "CO2 " + String(CO2ppm_value));

  // Print info
  switch (co2_device_status) {
    case ok:
      display.drawString(0, 24, "BIEN");
      break;
    case warning:
      display.drawString(0, 24, "REGULAR");
      break;
    case alarm:
      display.drawString(0, 24, "¡MAL!");
      break;
  }

  display.display();

}

// Update device status on OLED display
void update_OLED_Status() {

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, String(anaire_device_id));
  String ipaddress = WiFi.localIP().toString();
  display.drawString(0, 20, ipaddress);
  switch (global_device_status) {
    case device_ok:
      display.drawString(0, 40, "DEVICE OK");
      break;
    case err_wifi:
      display.drawString(0, 40, "ERR WIFI");
      break;
    case err_mqtt:
      display.drawString(0, 40, "ERR MQTT");
      break;
  }
  display.display();
}
