// 20201109 ANAIRE DEVICE CODE - Antonio López Gracia anaire@anaire.org
//
// Get CO2, temperature and humidity measurements and send them to the anaire cloud app
//
// Arduino Nano 33 IoT https://store.arduino.cc/arduino-nano-33-iot-with-headers?queryID=undefined
// MH-Z14A - CO2 sensor. Connected by serial port (serial1 on nano 33 iot) http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z14%20CO2%20V2.4.pdf
// AZ-Delivery DHT11 Temperature and humidity sensor https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
// AZ-Delivery LED semaphore - https://www.az-delivery.de/es/products/led-ampel-modul?_pos=5&_sid=5c7f89324&_ss=r
// AZ-Delivery Push button - https://www.az-delivery.de/es/products/button-modul?_pos=1&_sid=85dd3e4a9&_ss=r
// AZ-Delivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r

// edit the following file to configure the device: device id, wifi network and remote cloud endpoint
#include "anaire_config.h"

// Control Loop: time between measurements
const int CONTROL_LOOP_DURATION = 15000; // 15 seconds
unsigned long control_loop_start; // holds a timestamp for each control loop start

// WiFi
#include <SPI.h> // SPI Library
#include <WiFiNINA.h> // Wifi library for Arduino NANO 33 IoT
#define ARDUINO_NANO33IOT 1
int card = ARDUINO_NANO33IOT;
WiFiClient wifi_client;
int wifi_status = WL_IDLE_STATUS;
String firmware_version;
long wifi_rssi_dbm; // Received signal strength
IPAddress ip_address; // Wifi shield IP address
WiFiServer wifi_server(80); // to check if it is alive
char wifi_hostname[sizeof(anaire_device_id)];

// MQTT
#include <ArduinoMqttClient.h>
MqttClient mqttClient(wifi_client);
const char mqtt_send_topic[]  = "measurement";
const char mqtt_receive_topic[]  = "config";
char mqtt_message[200];

// HTTP
#include <ArduinoHttpClient.h> // Arduino http library
HttpClient http_client = HttpClient(wifi_client, cloud_server_address, cloud_app_port);
String http_path = "/metrics/job/" + (String)anaire_device_id;
String http_post_data; // for POST message
int http_status_code;
String http_response;

// MH-Z14A CO2 sensor
const unsigned long CO2_WARMING_TIME = 180000; // MH-Z14A CO2 sensor warming time: 3 minutes
byte measurement_command[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; // Command to get measurements from MH-Z14A CO2 sensor
byte calibration_command[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // Command to calibrate MH-Z14A CO2 sensor
char response_CO2[9]; // holds the received data from MH-Z14A CO2 sensor
int response_CO2_high; // holds upper byte
int response_CO2_low; // holds lower byte
int CO2ppm_value = 0; // CO2 ppm measured value

// AZ-Delivery DHT11
#include "DHT.h"
#define DHTPIN 20 // signal pin
#define DHTTYPE DHT11 // DHT (AM2302) Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//#define DHTTYPE DHT22   // DHT 22 (AM2302)
DHT dht_client(DHTPIN, DHTTYPE);
float temperature; // Read temperature as Celsius
float humidity; // Read humidity in %

// AZ-Delivery LED semaphore
int status_led_pin = 4;

// AZ-Delivery Push button
int push_button_pin = 3;
//int push_button_value = HIGH;

// AZ-Delivery Active Buzzer config
int buzzer_pin = 2;

// Status info
int builtin_led_pin = LED_BUILTIN; // builtin LED, used to provide visual insights to developers
int alarm_ack = false; // to indicate if push button has been pushed to ack the alarm and switch off the buzzer
enum Status {ok, warning, alarm}; // the device can have one of those status
Status device_status = ok; // initialized to ok

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

  // print info
  Serial.println();
  Serial.println();
  Serial.println("### INIT ANAIRE DEVICE ###########################################");

  // Check for the presence of the wifi shield
  if (card == ARDUINO_NANO33IOT) {
    if (WiFi.status() == WL_NO_MODULE) {
      Serial.println("WiFi shield not present - STOP!!!");
      while (true); // don't continue
    }
  }

  // Check Wifi firmware version
  firmware_version = WiFi.firmwareVersion();
  if (firmware_version < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to WiFi network:
  Connect_WiFi();
  Print_WiFi_Status(); // you're connected now, so print out the status

  // Attempt to connect to MQTT broker
  Init_MQTT();

  // initialize digital pin LED_BUILTIN as an output to blink, showing activity
  pinMode(builtin_led_pin, OUTPUT);

  // Status Led
  pinMode(status_led_pin, OUTPUT);
  digitalWrite(status_led_pin, LOW); // turn the LED off on boot

  // Buzzer
  pinMode(buzzer_pin, OUTPUT);

  // Push button
  pinMode(push_button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(push_button_pin), push_button_handler, FALLING);

  // Initialize serial port to communicate with MH-Z14A CO2 sensor. In Arduino nano 33 IoT this is Serial1
  Serial1.begin(9600);
  while (!Serial1) {
    Serial.print("Attempting to open serial port 1 to communicate to MH-Z14A CO2 sensor");
    delay(1000); // wait 1 seconds for connection
  }

  // Initialize MH-Z14A CO2 sensor: warming up and calibrate
  Setup_MHZ14A();

  // Initialize DHT11 temperature and humidity sensor
  dht_client.begin();

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  Serial.println ("--- LOOP BEGIN ---");

  // Timestamp for the loop start time
  control_loop_start = millis();

  // Turn off status LED to indicate the start of measurement and evaluation process
  digitalWrite(status_led_pin, HIGH);

  // Read MH-Z14A CO2 sensor
  Read_MHZ14A();

  // Evaluate CO2 value
  Evaluate_CO2_Value();

  // Read DHT sensor
  Read_Humidity_Temperature();

  // Message the cloud app to send the measured values
  Print_WiFi_Status();
  //Message_Cloud_App_HTTP(); // connect to http push gateway microservice in the cloud
  Message_Cloud_App_MQTT();   // connect to mqtt broker microservice in the cloud

  // Turn off status LED to indicate the end of measurement and evaluation process, only if there are not problems
  if (device_status != alarm) {
    digitalWrite(status_led_pin, LOW);
  }

  // Complete time up to ControlLoopTimerDuration and blink fast builtin LED to show it
  while ((millis() - control_loop_start) < CONTROL_LOOP_DURATION)
  {

    // Process wifi server requests
    Check_WiFi_Server();

    digitalWrite(builtin_led_pin, HIGH);
    delay(100);
    digitalWrite(builtin_led_pin, LOW);
    delay(100);
    
  }

  Serial.println("--- END LOOP");
  Serial.println();

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

// Connect to WiFi network
void Connect_WiFi() {
  while (wifi_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid); // print the network name (SSID);
    //anaire_device_id.toCharArray(wifi_hostname, sizeof(anaire_device_id));
    //WiFi.setHostname(wifi_hostname); // use anaire device id as hostname for easy wifi server access
    WiFi.setHostname(anaire_device_id); // use anaire device id as hostname for easy wifi server access
    wifi_status = WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
    delay(5000); // wait 5 seconds for connection
  }
  wifi_server.begin(); // start the web server on port 80
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
  //Serial.println(wl_status_to_string(WiFi.status());
  Serial.println(WiFi.status());
  
  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  ip_address = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip_address);

  // Print the received signal strength:
  wifi_rssi_dbm = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(wifi_rssi_dbm);
  Serial.println(" dBm");

  // print where to go in a browser:
  //Serial.print("To see this page in action, open a browser to http://");
  //Serial.println(ip_address);

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
            switch (device_status) {
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
  // Print info
  Serial.println ("Warming up MH-Z14A CO2 sensor...");
  // Wait for warming time
  delay(CO2_WARMING_TIME);
  // Print info
  Serial.println ("Calibrating MH-Z14A CO2 sensor...");
  // Write calibration command
  Serial1.write(calibration_command, 9);
  // Waits for 3 seconds
  delay(3000);
  // Print info
  Serial.println ("MH-Z14A CO2 sensor setup complete");
}

// Read MH-Z14A CO2 sensor
void Read_MHZ14A()
{
  // clears out any garbage in the RX buffer
  while (Serial1.available())
  {
    int garbage = Serial1.read();
    //Serial.println ("Cleaning serial1 data...");
  }
  // Send out read command to the sensor - 9 bytes
  Serial1.write(measurement_command, 9);
  // pauses the sketch and waits for the TX buffer to send all its data to the sensor
  Serial1.flush();
  // pauses the sketch and waits for the sensor response
  while (!Serial1.available())
  {
    delay(0);
    //Serial.println ("Waiting for serial1 data...");
  }
  // once data is available, it reads it to a variable 9 bytes
  Serial1.readBytes(response_CO2, 9);

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
    device_status = ok; // update device status
    noTone(buzzer_pin); // Switch off buzzer
    digitalWrite(status_led_pin, LOW); // Switch off status LED
    alarm_ack = false; // Init alarm ack status
  }

  // Status: warning
  else if ((CO2ppm_value >= CO2ppm_warning_threshold) && (CO2ppm_value < CO2ppm_alarm_threshold)) {
    device_status = warning; // update device status
    noTone(buzzer_pin); // Switch off buzzer
    digitalWrite(status_led_pin, LOW); // Switch off status LED
    alarm_ack = false; // Init alarm ack status
  }

  // Status: alarm
  else {
    device_status = alarm; // update device status
    // If the alarm is not yet ack, make buzzer sound and switch on status LED
    if (!alarm_ack) {
      // Sound frequency 3636 Hz (beautiful number)
      tone(buzzer_pin, 3636);
      // Start status led
      digitalWrite(status_led_pin, HIGH);
    }
  }

  // Print info
  switch (device_status) {
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

  // Read humidity as percentage
  humidity = dht_client.readHumidity();

  // Read temperature as Celsius
  temperature = dht_client.readTemperature();

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

// Send measurements to the cloud application by HTTP
void Message_Cloud_App_HTTP() {

  // Print info
  Serial.println("making POST request");

  // Prepare post_data
  http_post_data = "CO2 ";
  http_post_data += String(CO2ppm_value);
  http_post_data += "\nTemperature ";
  http_post_data += String(temperature, 6);
  http_post_data += "\nHumidity ";
  http_post_data += String(humidity, 6);
  http_post_data += "\n";
  Serial.println(http_post_data);

  // http POST send
  http_client.beginRequest();
  http_client.post(http_path);
  http_client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  http_client.sendHeader("Content-Length", http_post_data.length());
  http_client.sendHeader("X-Custom-Header", "custom-header-value");
  http_client.beginBody();
  http_client.print(http_post_data);
  http_client.endRequest();

  // read the status code and body of the response
  http_status_code = http_client.responseStatusCode();
  http_response = http_client.responseBody();

  // Print info
  Serial.print("Status code: ");
  Serial.println(http_status_code);
  Serial.print("Response: ");
  Serial.println(http_response);

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

  Serial.print("Sending mqtt message to the send topic");
  Serial.println(mqtt_send_topic);
  sprintf(mqtt_message, "{id: %s,CO2: %d, humidity: %f,temperature: %f}", anaire_device_id, CO2ppm_value, humidity, temperature);
  Serial.print(mqtt_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(mqtt_send_topic);
  mqttClient.print(mqtt_message);
  mqttClient.endMessage();

}

void push_button_handler() {
  if (!alarm_ack) {
    // Print info
    Serial.print ("Push button interrupt - alarm_ack");
    // Switch off the buzzer (but keeps the status LED on)
    noTone(buzzer_pin);
  }
  alarm_ack = true; // alarm has been ack
}


void Init_MQTT() {
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.print(cloud_server_address);
  Serial.print(":");
  Serial.println(cloud_app_port);
  if (!mqttClient.connect(cloud_server_address, cloud_app_port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
}

const char* wl_status_to_string(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";

    //WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
    //WL_IDLE_STATUS      = 0,
    //WL_NO_SSID_AVAIL    = 1,
    //WL_SCAN_COMPLETED   = 2,
    //WL_CONNECTED        = 3,
    //WL_CONNECT_FAILED   = 4,
    //WL_CONNECTION_LOST  = 5,
    //WL_DISCONNECTED     = 6
    
  }
}
