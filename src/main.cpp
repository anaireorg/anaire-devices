//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AireCiudadano medidor Fijo - Medidor de PM2.5 abierto, medición opcional de humedad y temperatura.
// Más información en: aireciudadano.com
// Este firmware es un fork del proyecto Anaire (https://www.anaire.org/) recomendado para la medición de CO2.
// 22/05/2022 info@aireciudadano.com
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pendientes:
//          OK: Nombre de la estacion con modelo de sensor, board  y etiqueta propia si es posible
//          OK: Programacion de modelo de sensor por portal cautivo
//          OK: BT funcionando en este codigo y sin WIFI y encendiendo sensor con pin enable
//          OK: TTGO T Display funcionando
//          OK: Agregar comparacion de valores de PM25 para emoticons y colores
//          OK: OLED funcionando
//          OK: AireCiudadano Splash Screen
//          OK: Valor de RSSI para modo wifi
//          OK: Añadir coordenadas GPS
//          OK: Añadir VOCs y NOx para SEN5X
//          OK: Revisar presicion envio de float, si cortarla o dejarlo al usuario
//          OK: Variable Sensor de exteriores o interiores, ExternalSensor via mqtt InOut inout
//          OK: Tiempo de muestreo
//          OK: Revisar diferencia entre Sensor characteristics y Numero ID de la configuracion del sensor
//          OK: Revisar Update por Portal Cautivo
//          OK: Guardar configuracion de sensores, board, display y demás en el Portal Cautivo
//          OK: Revisar como saber si una board tiene Brownout, si por config values
//          OK: Revision de Teclas para dormir en el Splash Screen!!!!!!!!!!!!!!!!!
//          OK: Revisión de Teclas para despertar, ojala fuera mas de 1 segundo por posibles ruidos de tecla
//          OK seguir revisando: Revisar la funcion de la APP de sample time a ver como se maneja desde el micro, investigar eso bien
//          OK: Version de firmware incluida en el valor IDn que se envia por la trama mqtt
//          OK: Arreglar problema con lectura Temperatura y Humedad SEN50 que generan errores por envio datos numericos altisimos (nan)
// Revisar actualizacion por orden a una direccion web repositorio y cada caso especifico: sin pantalla, OLED96, OLED66, wifi, bluetooth, etc
// SDy RTC version independiente o unido a BT y Wifi
//
// Revisar Datos guardados en Bluetooth con cada sleep, se pueden generar muchos datos: opcion preguntar si borrar antes de apagar, revisar esas opciones
//
// Version ESP8266:
//          Revisar porque muchas veces no carga bien el portal cautivo, queda en blanco la pagina. Parece ser si hay mas de 20 router por mostrar paila:
//          Aqui revisar: https://github.com/tzapu/WiFiManager/blob/master/WiFiManager.cpp
//
// Cambio en el archivo WifiEspClient.cpp:
// size_t WiFiEspClient::print(const __FlashStringHelper *ifsh)
//{
//	return printFSH(ifsh, false);
//}
//
// MODIFICACIONES EXTERNAS:
// Modificado WifiManager.cpp para que cuando ingrese al Config del portal cautivo pase a 180 segundos y no 10:
// _configPortalTimeout = 300000;   // New Config Portal Timeout
//  DEBUG_WM(DEBUG_VERBOSE,F("New Config Portal Timeout: 300 seconds"));
// Modificado template.h en texto: Configure WiFi por Configure WiFi and Sensor & Show Password

#include <Arduino.h>
#include "main.hpp"

////////////////////////////////
// Modo de comunicaciones del sensor:
#define Wifi true        // Set to true in case Wifi is desired, Bluetooth off and SDyRTCsave optional
#define Bluetooth false  // Set to true in case Bluetooth is desired, Wifi off and SDyRTCsave optional
#define SDyRTC false     // Set to true in case SD card and RTC (Real Time clock) is desires, Wifi and Bluetooth off
#define SaveSDyRTC false // Set to true in case SD card and RTC (Real Time clock) is desires to save data in Wifi or Bluetooth mode
#define ESP8285 false    // Set ti true in case you use a ESP8285 switch

// Escoger modelo de pantalla (pasar de false a true) o si no hay escoger ninguna (todas false):
#define Tdisplaydisp false
#define OLED66display false
#define OLED96display false

// Boards diferentes
#define TTGO_TQ false

// Definiciones opcionales para version Wifi
#define BrownoutOFF false   // Colocar en true en boards con problemas de RESET por Brownout o bajo voltaje
#define WPA2 true           // Colocar en true para redes con WPA2
#define PreProgSensor false // Variables de sensor preprogramadas:
                            // Latitude: char sensor_lat[10] = "xx.xxxx";
                            // Longitude: char sensor_lon[10] = "xx.xxxx";
                            // Valores de configuración: char ConfigValues[9] = "000xxxxx";
                            // Nombre estación: char aireciudadano_device_name[36] = "xxxxxxxxxxxxxx";

// Fin definiciones opcionales Wifi

bool SPS30sen = false;      // Sensor Sensirion SPS30
bool SEN5Xsen = false;      // Sensor Sensirion SEN5X
bool PMSsen = false;        // Sensor Plantower PMS
bool AdjPMS = false;        // PMS sensor adjust
bool SHT31sen = false;      // Sensor SHT31 humedad y temperatura
bool AM2320sen = false;     // Sensor AM2320 humedad y temperatura
bool TDisplay = false;      // Set to true if Board TTGO T-Display is used
bool OLED66 = false;        // Set to true if you use a OLED Diplay 0.66 inch 64x48
bool OLED96 = false;        // Set to true if you use a OLED Diplay 0.96 inch 128x64
bool ExtAnt = false;        // External antenna
bool AmbInOutdoors = false; // Set to true if your sensor is indoors measuring outside environment, false if is outdoors

uint8_t CustomValue = 0;
uint16_t CustomValtotal = 0;
char CustomValTotalString[9] = "00000000";
uint32_t IDn = 0;
String chipIdHEX;
uint32_t chipId = 0;

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen, in HEX format. Example: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String sw_version = "1.8";
String aireciudadano_device_id;
uint8_t Swver;

// Init to default values; if they have been chaged they will be readed later, on initialization
struct MyConfigStruct
{
#if Bluetooth
  uint16_t BluetoothTime = 10;        // Bluetooth Time
  char aireciudadano_device_name[30]; // Device name; default to aireciudadano_device_id
// #elif SDyRTC
//   uint16_t SDyRTCTime = 10; // SDyRTC Time
#elif Wifi
  uint16_t PublicTime = 1;                           // Publication Time
  uint16_t MQTT_port = 80;                           // MQTT port; Default Port on 80
  char MQTT_server[30] = "sensor.aireciudadano.com"; // MQTT server url or public IP address.
#if !PreProgSensor
  char sensor_lat[10] = "0.0";                       // Sensor latitude  GPS
  char sensor_lon[10] = "0.0";                       // Sensor longitude GPS
  char ConfigValues[10] = "000000000";
  char aireciudadano_device_name[30]; // Device name; default to aireciudadano_device_id
#else
  char sensor_lat[10] = "4.69375";   // Aquí colocar la Latitud del sensor
  char sensor_lon[10] = "-74.09382"; // Colocar la Longitud del sensor
  char ConfigValues[10] = "000010111";
  char aireciudadano_device_name[30] = "AireCiudadano_DBB_01"; // Nombre de la estacion
#endif
#endif
#if WPA2
  char wifi_user[24];     // WiFi user to be used on WPA Enterprise. Default to null (not used)
  char wifi_password[24]; // WiFi password to be used on WPA Enterprise. Default to null (not used)
#endif
} eepromConfig;

#if PreProgSensor
// const char *ssid = "Techotyva";
// const char *password = "Layuyux31";
const char *ssid = "TPred";
const char *password = "apt413sago16";
// const char *ssid = "Rosa";
// const char *password = "Rudysicha";
char aireciudadano_device_nameTemp[30] = {0};
#endif

#if BrownoutOFF
// OFF BROWNOUT/////////////////////
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#if ESP8266
// Save config values to EEPROM
#include <ESP_EEPROM.h>
#else
// to store data on nvs partition
#include <Preferences.h>
Preferences preferences;
#endif

// Measurements
float PM25_value = 0;           // PM25 measured value
float PM25_value_ori = 0;       // PM25 original measured value in PMS adjust TRUE
float PM25_accumulated = 0;     // Accumulates pm25 measurements for a MQTT period
float PM25_accumulated_ori = 0; // Accumulates pm25 measurements for a MQTT period in PMS Adjust TRUE
float temperature;              // Read temperature as Celsius
float humidity;                 // Read humidity in %
int PM25_samples = 0;           // Counts de number of samples for a MQTT period
int pm25int;                    // PM25 publicado
int pm25intori;

int temp;
int humi;

float latitudef = 0.0;
float longitudef = 0.0;

bool err_wifi = false;
bool err_MQTT = false;
bool err_sensor = false;
bool FlagDATAicon = false;
bool NoSensor = false;

// Measurements loop: time between measurements
unsigned int measurements_loop_duration = 1000; // 1 second
unsigned long measurements_loop_start;          // holds a timestamp for each control loop start

unsigned int Bluetooth_loop_time;
unsigned int Con_loop_times = 0;

unsigned int SDyRTC_loop_time;

// MQTT loop: time between MQTT measurements sent to the cloud
unsigned long MQTT_loop_start;          // holds a timestamp for each cloud loop start
unsigned long lastReconnectAttempt = 0; // MQTT reconnections

// Errors loop: time between error condition recovery
unsigned int errors_loop_duration = 60000; // 60 seconds
unsigned long errors_loop_start;           // holds a timestamp for each error loop start

#if !ESP8266
// TTGO ESP32 board
#include "esp_timer.h"
#include <esp_system.h>
#endif
#include <Wire.h>

// OLED display
unsigned int mcount, ecode = 0;
int lastDrawedLine = 0;
unsigned int inthumi = 0;
unsigned int inttemp = 0;
unsigned int cursor = 0;
bool toggleLive;
int dw = 0; // display width
int dh = 0; // display height

#if !ESP8266

#if OLED66display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
#elif OLED96display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
#elif Tdisplaydisp
#include "Iconos.h"
#endif

#else

#if OLED66display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 5, 4);
#elif OLED96display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 5, 4);
#endif

#endif

#if Tdisplaydisp
// Display and fonts
#include <TFT_eSPI.h>
#include <SPI.h>
#include "ArchivoNarrow_Regular50pt7b.h"
#include "ArimoBoldFont16.h"
#include "ArimoBoldFont20.h"
#define GFXFF 1
// #define FF90 &ArchivoNarrow_Regular10pt7b
#define FF90 &ArimoBoldFont16
#define FF92 &ArimoBoldFont20
#define FF95 &ArchivoNarrow_Regular50pt7b
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke library, pins defined in User_Setup.h

// Customized AireCiudadano splash screen
#include "Icono_AireCiudadano.h"

// Buttons: Top and bottom considered when USB connector is positioned on the right of the board

#include "Button2.h"
#define BUTTON_TOP 35
#define BUTTON_BOTTOM 0
Button2 button_top(BUTTON_TOP);
Button2 button_bottom(BUTTON_BOTTOM);

// Define ADC PIN for battery voltage measurement
#define ADC_PIN 34
float battery_voltage;
int vref = 1100;

// Define voltage threshold
#define USB_Voltage 4.0
#define Voltage_Threshold_1 3.9
#define Voltage_Threshold_2 3.7
#define Voltage_Threshold_3 3.5
#define Voltage_Threshold_4 3.3

#else

#define BUTTON_TOP 35   // ??????????
#define BUTTON_BOTTOM 0 // ??????????

#endif

#define Sensor_SDA_pin 21 // Define the SDA pin used for the SCD30
#define Sensor_SCL_pin 22 // Define the SCL pin used for the SCD30

#include <sps30.h>
SPS30 sps30;
#define SP30_COMMS Wire
#define DEBUG 0

#include <SensirionI2CSen5x.h>

// The used commands use up to 48 bytes. On some Arduino's the default buffer space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;
// bool SEN5Xflag = false;
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

#include "PMS.h"

#if !ESP8266

#if !TTGO_TQ

PMS pms(Serial1);
PMS::DATA data;
// bool PMSflag = false;
#define PMS_TX 17 // PMS TX pin
#define PMS_RX 15 // PMS RX pin

#else

PMS pms(Serial2);
PMS::DATA data;
// bool PMSflag = false;
#define PMS_TX 19 // PMS TX pin
#define PMS_RX 18 // PMS RX pin

#endif

#else
#include <SoftwareSerial.h>

#if !ESP8285
#define PMS_TX 14 // PMS TX pin
#define PMS_RX 12 // PMS RX pin

#else
#define PMS_TX 3  // PMS TX pin
#define PMS_RX 12 // PMS RX pin

#endif

SoftwareSerial pmsSerial(PMS_TX, PMS_RX); // SoftwareSerial(rxPin, txPin)

PMS pms(pmsSerial);
PMS::DATA data;
// bool PMSflag = false;

#endif

#include <Adafruit_SHT31.h>
Adafruit_SHT31 sht31;
bool SHT31flag = false;
byte failh = 0;

// Adafruit_SHT31 sht31 = Adafruit_SHT31();

#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
Adafruit_AM2320 am2320 = Adafruit_AM2320();
bool AM2320flag = false;

// Bluetooth in TTGO T-Display
#if Bluetooth
#include "Sensirion_GadgetBle_Lib.h" // to connect to Sensirion MyAmbience Android App available on Google Play
// GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_CO2_ALT);
GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_VOC_PM25_V2);
// bool bluetooth_active = false;
#endif

#if !ESP8266
#define OUT_EN 26 // Enable del elevador de voltaje
#else
#define OUT_EN 12 // Enable del elevador de voltaje
#endif

#if !ESP8266

#if Wifi
// WiFi
// #define WM_DEBUG_LEVEL DEBUG_DEV
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#if WPA2
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#endif

const int WIFI_CONNECT_TIMEOUT = 10000; // 10 seconds
// const int WIFI_CONNECT_TIMEOUT = 1000; // 1 seconds !!! TEST SEN5X
// const int captiveportaltime = 90;
WiFiServer wifi_server(80);
WiFiClient wifi_client;
bool PortalFlag = false;

WiFiManager wifiManager;

#endif

#else
#if Wifi

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

WiFiManager wifiManager;

// WiFi
#include <ESP8266WiFi.h> // Wifi ESP8266
extern "C"
{
#include "user_interface.h"

#if WPA2
#include "wpa2_enterprise.h"
#endif

#include "c_types.h"
  bool PortalFlag = false;
}

#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
WiFiClient wifi_client;
const int WIFI_CONNECT_TIMEOUT = 10000; // 10 seconds
int wifi_status = WL_IDLE_STATUS;
WiFiServer wifi_server(80); // to check if it is alive
                            // String wifi_ssid = WiFi.SSID();                  // your network SSID (name)
                            // String wifi_password = WiFi.psk();               // your network psk password

#include <ESP8266WebServer.h>
#include <DNSServer.h>

#endif

#endif

#if Wifi

// MQTT
#include <PubSubClient.h>
char MQTT_message[256];
PubSubClient MQTT_client(wifi_client);
char received_payload[384];
String MQTT_send_topic;
String MQTT_receive_topic;

// JSON
#include <ArduinoJson.h>
StaticJsonDocument<384> jsonBuffer;

#if !ESP8266
// OTA Update
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#else
// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#endif

#endif

#if !ESP8266
#include "rom/rtc.h"
#endif

bool ResetFlag = false;
bool DeepSleepFlag = false;
bool NoiseBUTTONFlag = false;
// int reason;

#if !ESP8266
void print_reset_reason(RESET_REASON reason);
#endif

// to know when there is an updating process in place
bool updating = false;

// To know when the device is in the following states
bool InCaptivePortal = false;
bool Calibrating = false;

#if (SDyRTC || SaveSDyRTC)

#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

const int chipSelect = 10;
uint16_t SDyRTCtime = 15;

File dataFile;

RTC_DS1307 rtc;

#define LEDPIN 2

#endif

#if ESP8285
#define LEDPIN 13
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(115200);
  delay(100);
  while (!Serial)
  {
    delay(500); // wait 0.5 seconds for connection
  }
  Serial.setDebugOutput(true);

#if Wifi

#if !ESP8266
  Serial.println(F("CPU0 reset reason:"));
  print_reset_reason(rtc_get_reset_reason(0));
#else
  uint16_t Resetvar = 0;
  Serial.print(F("CPU reset reason: "));
  rst_info *rinfo = ESP.getResetInfoPtr();
  Serial.println(rinfo->reason);
  Resetvar = rinfo->reason;
  ResetFlag = true;
  if (Resetvar == 4)
  {
    ResetFlag = false;
    Serial.print(F("Resetvar: false"));
  }
  Serial.print(F("Resetvar: "));
  Serial.println(Resetvar);
#endif

#endif

#if Bluetooth
  if (DeepSleepFlag == true)
  {
    delay(100);
    if (digitalRead(BUTTON_TOP) == false)
    {
      delay(900);
      if (digitalRead(BUTTON_TOP) == false)
      {
        NoiseBUTTONFlag = false;
        Serial.println(F("NoiseBUTTONFlag = false"));
      }
      else
      {
        NoiseBUTTONFlag = true;
        Serial.println(F("NoiseBUTTONFlag = true"));
        Suspend_Device();
      }
    }
    else
    {
      NoiseBUTTONFlag = true;
      Serial.println(F("NoiseBUTTONFlag = true"));
      Suspend_Device();
    }
  }
#endif

  // print info
  Serial.println();
  Serial.println(F("##### Inicializando Medidor Aire Ciudadano #####"));

#if !ESP8266
  // init preferences to handle persitent config data
  preferences.begin("config"); // use "config" namespace
#endif

#if PreProgSensor
  Serial.print(F("T1: "));
  Serial.println(eepromConfig.aireciudadano_device_name);
  strcpy(aireciudadano_device_nameTemp, eepromConfig.aireciudadano_device_name);
#endif
#if !SDyRTC
  // Read EEPROM config values
  Read_EEPROM();
#endif
#if PreProgSensor
  strncpy(eepromConfig.aireciudadano_device_name, aireciudadano_device_nameTemp, sizeof(eepromConfig.aireciudadano_device_name));
  Serial.print(F("T2:"));
  Serial.println(eepromConfig.aireciudadano_device_name);
#endif

#if ESP8285
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
#endif

#if !SDyRTC
  aireciudadano_device_id = eepromConfig.aireciudadano_device_name;
#endif

  float Floatver = sw_version.toFloat();
  Swver = Floatver * 10;
  Serial.print(F("SW version: "));
  Serial.println(sw_version);

  // Get device id
  Get_AireCiudadano_DeviceId();

#if Bluetooth
  Bluetooth_loop_time = eepromConfig.BluetoothTime;
  gadgetBle.setSampleIntervalMs(Bluetooth_loop_time * 1000); // Valor de muestreo de APP y de Sensor
// #elif SDyRTC
//   SDyRTC_loop_time = eepromConfig.SDyRTCTime;
#endif

#if Tdisplaydisp
  TDisplay = true;
  OLED66 = false;
  OLED96 = false;
#elif OLED66display
  TDisplay = false;
  OLED66 = true;
  OLED96 = false;
#elif OLED96display
  TDisplay = false;
  OLED66 = false;
  OLED96 = true;
#else
  TDisplay = false;
  OLED66 = false;
  OLED96 = false;
#endif

#if BrownoutOFF
  // OFF BROWNOUT/////////////////////
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable   detector
#endif

#if (Tdisplaydisp || OLED96display || OLED66display)

  if (TDisplay == true)
  {
#if !ESP8266
    // Initialize TTGO Display and show AireCiudadano splash screen
    Button_Init();
    Display_Init();
    Display_Splash_Screen();

    for (int i = 0; i < 10; i++)
    {
#if Tdisplaydisp
      button_top.loop();
#endif
      delay(500);
    }
#endif
  }
  else if (OLED66 == true || OLED96 == true)
  {
    pinMode(BUTTON_BOTTOM, INPUT_PULLUP);
#if !Tdisplaydisp
    displayInit();
    pageStart();
    showWelcome();
    delay(100);
    u8g2.drawXBM(16, 18, 32, 32, IconoAC);
    pageEnd();
#endif
    delay(3000);
  }

#else
  pinMode(BUTTON_BOTTOM, INPUT_PULLUP);
  delay(100);

#endif

  // Out for power on and off sensors
  pinMode(OUT_EN, OUTPUT);
  // On sensors
  digitalWrite(OUT_EN, HIGH); // step-up on
  delay(100);

#if Wifi
  // Set MQTT topics
  MQTT_send_topic = "measurement"; // measurement are sent to this topic
  // MQTT_send_topic = "measurementfix";                          // measurementfix are sent to this topic
  MQTT_receive_topic = "config/" + aireciudadano_device_id; // Config messages will be received in config/id
#endif

  // Print initial configuration
  Print_Config();

#if Wifi
  // Set Latitude and Longitude
  latitudef = atof(eepromConfig.sensor_lat);
  longitudef = atof(eepromConfig.sensor_lon);

// Initialize the GadgetBle Library for Bluetooth
#elif Bluetooth
  gadgetBle.begin();
  Serial.print(F("Sensirion GadgetBle Lib initialized with deviceId = "));
  Serial.println(gadgetBle.getDeviceIdString());

#endif

#if Wifi
  // Start Captive Portal for 60 seconds
  if (ResetFlag == true)
  {
    Start_Captive_Portal();
    delay(100);
  }

  // Attempt to connect to WiFi network:
  Connect_WiFi();

  // Attempt to connect to MQTT broker
  if (!err_wifi)
  {
    Init_MQTT();
#if ESP8285
    digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
    delay(750);                // wait for a 750 msecond
    digitalWrite(LEDPIN, HIGH);
#endif
  }
#endif

  // Initialize and warm up PM25 sensor
  Setup_Sensor();

  // Init control loops
  measurements_loop_start = millis();
  errors_loop_start = millis();
#if Wifi
  MQTT_loop_start = millis();
#endif

#if (SDyRTC || SDyRTC)

  pinMode(LEDPIN, OUTPUT);

  Serial.print(F("Initializing SD card: ");
  // make sure that the default chip select pin is set to output, even if you don't use it:
  pinMode(SS, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println(F("Card failed, or not present"));
    Serial.println(F("Review SD card or connections"));
  }
  else
  {
    Serial.println(F("OK, card initialized"));

    // Open up the file we're going to log to!
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (!dataFile)
    {
      Serial.println(F("error opening datalog.txt"));
      Serial.println(F("Review the SD card"));
    }
    else
    {
      digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
      delay(500);                 // wait for a 500 msecond
      digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
      delay(500);                 // wait for a 500 msecond
      digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
      delay(500);                 // wait for a 500 msecond
      digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
      delay(500);                 // wait for a 500 msecond
    }
  }

  Serial.print(F("Initializing RTC ds1307: "));

  if (!rtc.begin())
  {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
  }
  else
  {
    Serial.println(F("OK, ds1307 initialized"));
    digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
    delay(200);                 // wait for a 500 msecond
    digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
    delay(200);                 // wait for a 500 msecond
    digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
    delay(200);                 // wait for a 500 msecond
    digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
  }

  if (!rtc.isrunning())
  {
    Serial.println(F("RTC is NOT running, let's set the time!"));
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2022, 8, 20, 15, 18, 0));
  }
  else
    Serial.println(F("ds1307 is running, no changes"));

    //  rtc.adjust(DateTime(2022, 8, 20, 15, 18, 0));

#endif

  Serial.println(F(""));
  Serial.println(F("### Configuración del medidor AireCiudadano finalizada ###\n"));

#if (Tdisplaydisp || OLED96display || OLED66display)

  if (TDisplay == true)
  {
#if !ESP8266
#if Tdisplaydisp
    tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Medidor", tft.width() / 2, (tft.height() / 2) - 30);
    tft.drawString("AireCiudadano", tft.width() / 2, tft.height() / 2);
    tft.drawString("ver: " + String(), tft.width() / 2 - 20, (tft.height() / 2) + 30);
    tft.drawString(sw_version.c_str(), tft.width() / 2 + 20, (tft.height() / 2) + 30);
    delay(2000);
    // Update display with new values
    Update_Display();
#endif
#endif
  }
  else if (OLED66 == true || OLED96 == true)
  {
#if !Tdisplaydisp
    pageStart();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.setCursor(0, (dh / 2 - 4));
    u8g2.print("Medidor Listo");
    delay(1000);
    pageEnd();
#endif
  }

#endif

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // If a firmware update is in progress do not do anything else
  if (updating)
  {
    return;
  }
#if Bluetooth
#if Tdisplaydisp
  // Measure the battery voltage
  battery_voltage = ((float)analogRead(ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
#endif
#endif

  // Measurement loop
  if ((millis() - measurements_loop_start) >= measurements_loop_duration)
  {

    // New timestamp for the loop start time
    measurements_loop_start = millis();

    // Read sensors
    Read_Sensor();

    if (NoSensor == false)
    {
      if (PM25_value >= 0)
      {

#if (Tdisplaydisp || OLED96display || OLED66display)

        // Update display with new values
        if (TDisplay == true)
        {
#if !ESP8266
#if Tdisplaydisp
          Update_Display();
#endif
#endif
        }
        else if (OLED66 == true || OLED96 == true)
        {
#if !Tdisplaydisp
          UpdateOLED();
#endif
        }

#else
#if Bluetooth
        TimeConfig();
#endif
#endif

        // Accumulates samples
        PM25_accumulated += PM25_value;
        if (AdjPMS == true)
          PM25_accumulated_ori += PM25_value_ori;
        PM25_samples++;
        Con_loop_times++;
      }
    }
    else
    {
      Serial.println(F("Medidor No configurado"));

#if (Tdisplaydisp || OLED96display || OLED66display)

      if (TDisplay == true)
      {
#if !ESP8266
#if Tdisplaydisp
        tft.fillScreen(TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.setTextDatum(6); // bottom left
        tft.setTextSize(1);
        tft.setFreeFont(FF90);
        tft.setTextDatum(MC_DATUM);
        if (Bluetooth == true)
        {
          tft.drawString("Sensores", tft.width() / 2, (tft.height() / 2) - 30);
          tft.drawString("No conectados", tft.width() / 2, (tft.height() / 2) + 20);
        }
        else
        {
          tft.drawString("Medidor", tft.width() / 2, (tft.height() / 2) - 30);
          tft.drawString("No configurado", tft.width() / 2, (tft.height() / 2) + 20);
        }
        delay(1000);
#endif
#endif
      }
      else if (OLED66 == true || OLED96 == true)
      {

#if !Tdisplaydisp
        pageStart();
        u8g2.setFont(u8g2_font_5x8_tf);
        if (Bluetooth == true)
        {
          u8g2.setCursor(4, (dh / 2 - 7));
          u8g2.print("Sensores No");
          u8g2.setCursor(8, (dh / 2 + 7));
          u8g2.print("conectados");
        }
        else
        {
          u8g2.setCursor(8, (dh / 2 - 7));
          u8g2.print("Medidor No");
          u8g2.setCursor(5, (dh / 2 + 7));
          u8g2.print("configurado");
        }
        pageEnd();
#endif
        delay(2000);
      }

#endif
    }
  }

#if Bluetooth

  // Bluetooth loop

  if (Con_loop_times >= eepromConfig.BluetoothTime)
  {
    float PM25f;

    ///// DEBUG Samples
    // Serial.println(F(PM25_accumulated));
    // Serial.print(F("#samples: "));
    // Serial.println(F(PM25_samples));
    PM25f = PM25_accumulated / PM25_samples;
    pm25int = round(PM25f);
    // Serial.println(F(pm25int));
    ///// END DEBUG Samples
    Serial.print(F("PM25: "));
    Serial.print(pm25int);
    Serial.print(F("   "));
    ReadHyT();
    Write_Bluetooth();
#if SaveSDyRTC
    Write_SD();
#endif
    PM25_accumulated = 0.0;
    PM25_samples = 0.0;
    Con_loop_times = 0;
  }
#elif SDyRTC

  // SDyRTC loop

  if (Con_loop_times >= SDyRTCtime)
  {
    float PM25f;

    ///// DEBUG Samples
    // Serial.println(F(PM25_accumulated));
    // Serial.print(F("#samples: "));
    // Serial.println(F(PM25_samples));
    PM25f = PM25_accumulated / PM25_samples;
    pm25int = round(PM25f);
    // Serial.println(F(pm25int));
    ///// END DEBUG Samples
    Serial.print(F("PM25: "));
    Serial.print(pm25int);
    Serial.print(F("   "));
    ReadHyT();
    Write_SD();
    PM25_accumulated = 0.0;
    PM25_samples = 0.0;
    Con_loop_times = 0;
  }

#else
  // MQTT loop
  if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 60000))
  {

    // New timestamp for the loop start time
    MQTT_loop_start = millis();

    // Message the MQTT broker in the cloud app to send the measured values
    if ((!err_wifi) && (PM25_samples > 0))
    {
      Send_Message_Cloud_App_MQTT();
    }

#if SaveSDyRTC
    Write_SD();
#endif

    // Reset samples after sending them to the MQTT server
    PM25_accumulated = 0.0;
    PM25_accumulated_ori = 0.0;
    PM25_samples = 0.0;
  }
#endif

  // Errors loop
  if ((millis() - errors_loop_start) >= errors_loop_duration)
  {

    // New timestamp for the loop start time
    errors_loop_start = millis();

    // Try to recover error conditions
    if (err_sensor)
    {
      Serial.println(F("--- err_sensor"));
      // Setup_Sensor();  // Init pm25 sensors
    }

#if Wifi

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println(F("--- err_wifi"));
      err_wifi = true;
      WiFi.reconnect();
    }
    else
    {
      err_wifi = false;
    }

    // Reconnect MQTT if needed
    if ((!MQTT_client.connected()) && (!err_wifi))
    {
      Serial.println(F("--- err_mqtt"));
      err_MQTT = true;
      FlagDATAicon = false;
    }

    // Reconnect MQTT if needed
    if ((err_MQTT) && (!err_wifi))
    {
      Serial.println(F("--- MQTT reconnect"));
      // Attempt to connect to MQTT broker
      MQTT_Reconnect();
      Init_MQTT();
    }
#endif
  }

#if Wifi

  // From here, all other tasks performed outside of measurements, MQTT and error loops

  // if not there are not connectivity errors, receive MQTT messages
  if ((!err_MQTT) && (!err_wifi))
  {
    MQTT_client.loop();
  }

  // Process wifi server requests
  Check_WiFi_Server();

#endif

// Process Bluetooth events
#if Bluetooth
  gadgetBle.handleEvents();
  delay(3);
#endif

#if !ESP8266
#if Tdisplaydisp
  if (TDisplay == true)
  {
    // Process buttons events
    button_top.loop();
    button_bottom.loop();
  }
#endif
#endif

  // Serial.println(F("--- END LOOP"));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

#if Wifi

#if !ESP8266

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d - ", event));

  switch (event)
  {
  case SYSTEM_EVENT_WIFI_READY:
    Serial.println(F("WiFi interface ready"));
    break;
  case SYSTEM_EVENT_SCAN_DONE:
    Serial.println(F("Completed scan for access points"));
    break;
  case SYSTEM_EVENT_STA_START:
    Serial.println(F("WiFi client started"));
    break;
  case SYSTEM_EVENT_STA_STOP:
    Serial.println(F("WiFi clients stopped"));
    break;
  case SYSTEM_EVENT_STA_CONNECTED:
    Serial.println(F("Connected to access point"));
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println(F("Disconnected from WiFi access point"));
    break;
  case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    Serial.println(F("Authentication mode of access point has changed"));
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.print(F("Obtained IP address: "));
    Serial.println(F(WiFi.localIP()));
    break;
  case SYSTEM_EVENT_STA_LOST_IP:
    Serial.println(F("Lost IP address and IP address is reset to 0"));
    break;
  case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
    Serial.println(F("WiFi Protected Setup (WPS): succeeded in enrollee mode"));
    break;
  case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    Serial.println(F("WiFi Protected Setup (WPS): failed in enrollee mode"));
    break;
  case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
    Serial.println(F("WiFi Protected Setup (WPS): timeout in enrollee mode"));
    break;
  case SYSTEM_EVENT_STA_WPS_ER_PIN:
    Serial.println(F("WiFi Protected Setup (WPS): pin code in enrollee mode"));
    break;
  case SYSTEM_EVENT_AP_START:
    Serial.println(F("WiFi access point started"));
    break;
  case SYSTEM_EVENT_AP_STOP:
    Serial.println(F("WiFi access point  stopped"));
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    Serial.println(F("Client connected"));
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    Serial.println(F("Client disconnected"));
    break;
  case SYSTEM_EVENT_AP_STAIPASSIGNED:
    Serial.println(F("Assigned IP address to client"));
    break;
  case SYSTEM_EVENT_AP_PROBEREQRECVED:
    Serial.println(F("Received probe request"));
    break;
  case SYSTEM_EVENT_GOT_IP6:
    Serial.println(F("IPv6 is preferred"));
    break;
  case SYSTEM_EVENT_ETH_START:
    Serial.println(F("Ethernet started"));
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println(F("Ethernet stopped"));
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println(F("Ethernet connected"));
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println(F("Ethernet disconnected"));
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.println(F("Obtained IP address"));
    break;
  default:
    break;
  }
}

#else
// Print wifi status on serial monitor
// ESP8266

void Print_WiFi_Status_ESP8266()
{

  // Get current status
  //  WL_CONNECTED: assigned when connected to a WiFi network;
  //  WL_NO_SHIELD: assigned when no WiFi shield is present;
  //  WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
  //  WL_NO_SSID_AVAIL: assigned when no SSID are available;
  //  WL_SCAN_COMPLETED: assigned when the scan networks is completed;
  //  WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
  //  WL_CONNECTION_LOST: assigned when the connection is lost;
  //  WL_DISCONNECTED: assigned when disconnected from a network;

  // wifi_status = WiFi.status();
  Serial.print(F("wifi_status: "));
  Serial.println(WiFi.status());

  // Print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  // ip_address = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(WiFi.localIP());

  // Print your WiFi shield's MAC address:
  Serial.print(F("MAC Adress: "));
  Serial.println(WiFi.macAddress());

  // Print the received signal strength:
  // wifi_rssi_dbm = WiFi.RSSI();
  Serial.print(F("Signal strength (RSSI):"));

  // #if !ESP8266
  Serial.print(WiFi.RSSI());
  // #else
  //   Serial.print(WiFi.RSSI());
  // #endif

  Serial.println(F(" dBm"));
}

#endif

void Connect_WiFi()
{ // Connect to WiFi

  WiFi.disconnect(true); // disconnect from wifi to set new wifi connection
  WiFi.mode(WIFI_STA);   // init wifi mode

#if !ESP8266

  WiFi.onEvent(WiFiEvent);

#if !PreProgSensor
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_STA, &conf); // Get WiFi configuration
  Serial.print(F("Attempting to connect to WiFi network: "));
  Serial.println(String(reinterpret_cast<const char *>(conf.sta.ssid))); // WiFi.SSID() is not filled up until the connection is established
#endif

#endif

#if !WPA2

#if ESP8266
#if !PreProgSensor
  WiFi.begin();
#else
  WiFi.begin(ssid, password);
#endif
#endif

#else
  // #if WPA2
  //  If there are not wifi user and wifi password defined, proceed to traight forward configuration

  if ((strlen(eepromConfig.wifi_user) == 0) && (strlen(eepromConfig.wifi_password) == 0))
  {
    Serial.println(F("Attempting to authenticate..."));

#if ESP8266

#if !PreProgSensor
    WiFi.begin();
#else
    WiFi.begin(ssid, password);
#endif

#endif
  }
  else
  {
#if WPA2
    // set up wpa2 enterprise
#if !ESP8266
    Serial.println(F("Attempting to authenticate using WPA2 Enterprise..."));
    Serial.print(F("User: "));
    Serial.println(eepromConfig.wifi_user);
    Serial.print(F("Password: "));
    Serial.println(eepromConfig.wifi_password);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));         // provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));         // provide username --> identity and username is same
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)eepromConfig.wifi_password, strlen(eepromConfig.wifi_password)); // provide password
    esp_wifi_sta_wpa2_ent_enable();

#else

    String wifi_ssid = WiFi.SSID(); // your network SSID (name)
    // String wifi_password = WiFi.psk()); // your network psk password
    Serial.print(F("Attempting to authenticate with WPA2 Enterprise "));
    Serial.print(F("User: "));
    Serial.println(eepromConfig.wifi_user);
    Serial.print(F("Password: "));
    Serial.println(eepromConfig.wifi_password);

    // Setting ESP into STATION mode only (no AP mode or dual mode)
    wifi_set_opmode(STATION_MODE);

    struct station_config wifi_config;

    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char *)wifi_config.ssid, wifi_ssid.c_str());
    strcpy((char *)wifi_config.password, eepromConfig.wifi_password);

    wifi_station_set_config(&wifi_config);
    // uint8_t target_esp_mac[6] = {0x24, 0x0a, 0xc4, 0x9a, 0x58, 0x28};
    // wifi_set_macaddr(STATION_IF,target_esp_mac);
    wifi_station_set_wpa2_enterprise_auth(1);

    // Clean up to be sure no old data is still inside
    wifi_station_clear_cert_key();
    wifi_station_clear_enterprise_ca_cert();
    wifi_station_clear_enterprise_identity();
    wifi_station_clear_enterprise_username();
    wifi_station_clear_enterprise_password();
    wifi_station_clear_enterprise_new_password();

    // Set up authentication
    wifi_station_set_enterprise_identity((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_username((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_password((uint8 *)eepromConfig.wifi_password, strlen((char *)eepromConfig.wifi_password));

    wifi_station_connect();
#endif

#endif
  }
  // #endif
#endif

#if ESP32

#if !PreProgSensor
  WiFi.begin();
#else
  WiFi.begin(ssid, password);
#endif

#endif

  // Timestamp for connection timeout
  int wifi_timeout_start = millis();

  // Wait for warming time while blinking blue led
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifi_timeout_start) < WIFI_CONNECT_TIMEOUT))
  {
    delay(500); // wait 0.5 seconds for connection
    Serial.println(F("."));
  }

  // Status
  if (WiFi.status() != WL_CONNECTED)
  {
    err_wifi = true;
    Serial.println(F("WiFi not connected"));
  }
  else
  {
    err_wifi = false;
    Serial.println(F("WiFi connected"));
    // start the web server on port 80
    wifi_server.begin();
  }

#if ESP8266
  Print_WiFi_Status_ESP8266();
#else
  Print_WiFi_Status();
#endif
}

void Print_WiFi_Status()
{ // Print wifi status on serial monitor

  // Get current status
  //  WL_CONNECTED: assigned when connected to a WiFi network;
  //  WL_NO_SHIELD: assigned when no WiFi shield is present;
  //  WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
  //  WL_NO_SSID_AVAIL: assigned when no SSID are available;
  //  WL_SCAN_COMPLETED: assigned when the scan networks is completed;
  //  WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
  //  WL_CONNECTION_LOST: assigned when the connection is lost;
  //  WL_DISCONNECTED: assigned when disconnected from a network;

  Serial.println(F("wifi_status: "));
  Serial.println(F(""));
#if (OLED66 == true || OLED96 == true)
  pageStart();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(10, dh / 2);
#endif
  switch (WiFi.status())
  {
  case WL_CONNECTED:
    Serial.println(F("WIFI CONECTADA !!!!!!!!!!!!!!!!!!!!"));
    Serial.println(F("WIFI CONECTADA !!!!!!!!!!!!!!!!!!!!"));
#if (OLED66 == true || OLED96 == true)
    u8g2.print("OK WIFI :)");
#endif
    break;
  case WL_NO_SHIELD:
    Serial.println(F("No WiFi HW detected"));
    break;
  case WL_IDLE_STATUS:
    Serial.println(F("Attempting..."));
    break;
  case WL_NO_SSID_AVAIL:
    Serial.println(F("No SSID available"));
#if (OLED66 == true || OLED96 == true)
    u8g2.print("NO WIFI :(");
#endif
    break;
  case WL_SCAN_COMPLETED:
    Serial.println(F("Networks scan completed"));
    break;
  case WL_CONNECT_FAILED:
    Serial.println(F("Connect failed"));
#if (OLED66 == true || OLED96 == true)
    u8g2.print("NO WIFI :(");
#endif
    break;
  case WL_CONNECTION_LOST:
    Serial.println(F("Connection lost"));
#if (OLED66 == true || OLED96 == true)
    u8g2.print("NO WIFI :(");
#endif
    break;
  case WL_DISCONNECTED:
    Serial.println(F("Disconnected"));
#if (OLED66 == true || OLED96 == true)
    u8g2.print("NO WIFI :(");
#endif
    break;
  default:
    Serial.println(F("Unknown status"));
    break;
  }
  Serial.println(F(""));
  delay(3000);
#if (OLED66 == true || OLED96 == true)
  pageEnd();
#endif

  // Print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  Serial.print(F("IP Address: "));
  Serial.println(WiFi.localIP());

  // Print your WiFi shield's MAC address:
  Serial.print(F("MAC Adress: "));
  Serial.println(WiFi.macAddress());

  // Print the received signal strength:
  Serial.print(F("Signal strength (RSSI): "));

  // #if !ESP8266
  Serial.print(WiFi.RSSI());
  // #else
  //   Serial.print(WiFi.RSSI());
  // #endif

  Serial.println(F(" dBm"));
}

void Check_WiFi_Server()
{                                              // Wifi server
  WiFiClient client = wifi_server.available(); // listen for incoming clients
  if (client)
  {                               // if you get a client,
    Serial.println(F("new client")); // print a message out the serial port
    String currentLine = "";      // make a String to hold incoming data from the client
    while (client.connected())
    { // loop while the client's connected
      if (client.available())
      {                         // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        Serial.write(c);        // print it out the serial monitor
        if (c == '\n')
        { // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // Print current info
            client.print("Medidor AireCiudadano");
            client.println("<br>");
            client.print("SW version: ");
            client.print(sw_version);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("AireCiudadano Device ID: ");
            client.print(aireciudadano_device_id);
            client.println("<br>");
            client.print("AireCiudadano custom name: ");
            client.print(eepromConfig.aireciudadano_device_name);
            client.println("<br>");
            client.print("SSID: ");
            client.print(String(WiFi.SSID()));
            client.println("<br>");
            client.print("IP Adress: ");
            client.print(WiFi.localIP());
            client.println("<br>");
            client.print("MAC Adress: ");
            client.print(WiFi.macAddress());
            client.println("<br>");
            client.print("RSSI: ");
            // #if !ESP8266
            client.print(WiFi.RSSI());
            // #else
            //             client.print(WiFi.RSSI());
            // #endif

            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("Publication Time: ");
            client.print(eepromConfig.PublicTime);
            client.print("MQTT Server: ");
            client.print(eepromConfig.MQTT_server);
            client.println("<br>");
            client.print("MQTT Port: ");
            client.print(eepromConfig.MQTT_port);
            client.println("<br>");
            client.print("Sensor latitude: ");
            client.print(eepromConfig.sensor_lat);
            client.println("<br>");
            client.print("Sensor longitude: ");
            client.print(eepromConfig.sensor_lon);
            client.println("<br>");
            //            client.print("Alarm: ");
            //            client.print(eepromConfig.acoustic_alarm);
            //            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("PM2.5: ");
            client.print(PM25_value);
            client.println("<br>");
            client.print("Temperature: ");
            client.print(temp);
            client.println("<br>");
            client.print("Humidity: ");
            client.print(humi);
            client.println("<br>");
            client.println("------");
            client.println("<br>");

            // Captive portal:
            client.print("Click <a href=\"/3\">here</a> to launch captive portal to set up WiFi and MQTT endpoint.<br>");
            client.println("<br>");
            // Suspend:
            client.print("Click <a href=\"/4\">here</a> to suspend the device.<br>");
            client.println("<br>");
            // Restart:
            client.print("Click <a href=\"/5\">here</a> to restart the device.<br>");
            client.println("<br>");

            // The HTTP response ends with another blank line:
            client.println();

            // break out of the while loop:
            break;
          }
          else
          { // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r')
        {                   // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /1" to calibrate the sensor:

        // Check to see if the client request was "GET /3" to launch captive portal:
        if (currentLine.endsWith("GET /3"))
        {
          PortalFlag = true;
          Start_Captive_Portal();
        }
#if !ESP8266
        // Check to see if the client request was "GET /4" to suspend the device:
        if (currentLine.endsWith("GET /4"))
        {
          Suspend_Device();
        }
#endif

        // Check to see if the client request was "GET /5" to restart the device:
        if (currentLine.endsWith("GET /5"))
        {
          ESP.restart();
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println(F("client disconnected"));
  }
}

void Start_Captive_Portal()
{ // Run a captive portal to configure WiFi and MQTT
  InCaptivePortal = true;
  String wifiAP;
  const int captiveportaltime = 60;
//  const int captiveportaltime = 5;

  //  wifiAP = String(eepromConfig.aireciudadano_device_name);

  //  if (wifiAP.isEmpty())
  //    wifiAP = "AireCiudadano_" + aireciudadano_device_id;
  //  else
  wifiAP = aireciudadano_device_id;
  Serial.println(wifiAP);

#if (Tdisplaydisp || OLED96display || OLED66display)
#if !ESP8266
#if Tdisplaydisp

  if (TDisplay == true)
  {
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Portal Cautivo", tft.width() / 2, (tft.height() / 2 - 20));
    tft.drawString(wifiAP, tft.width() / 2, tft.height() / 2 + 10);
  }
#endif
#endif

  if (OLED66 == true || OLED96 == true)
  {
#if !Tdisplaydisp
    pageStart();
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.setCursor(2, (dh / 2) - 10);
    u8g2.print("Portal cautivo");

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(5, dh / 2);
    u8g2.print(captiveportaltime);
    u8g2.setCursor(20, dh / 2);
    u8g2.print(" segundos");
    //    delay(2000);
    pageEnd();

#endif
  }

#endif

  wifi_server.stop();

  wifiManager.setDebugOutput(true);
  wifiManager.disconnect();

  // #if ESP8266
  //   wifiManager.setMinimumSignalQuality(50);
  // #endif

  WiFi.mode(WIFI_AP); // explicitly set mode, esp defaults to STA+AP

  // Captive portal parameters

#if WPA2
  WiFiManagerParameter custom_wifi_html("<p>Set WPA2 Enterprise</p>"); // only custom html
  WiFiManagerParameter custom_wifi_user("User", "WPA2 Enterprise user-identity", eepromConfig.wifi_user, 24);
  WiFiManagerParameter custom_wifi_password("Password", "WPA2 Enterprise Password", eepromConfig.wifi_password, 24);
  WiFiManagerParameter custom_wifi_html2("<p></p>"); // only custom html
#endif

#if !ESP8266
  WiFiManagerParameter custom_id_name("CustomName", "Set Station Name (29 characters max):", eepromConfig.aireciudadano_device_name, 29);
#else
  WiFiManagerParameter custom_id_name("CustomName", "Set Station Name (25 characters max):", eepromConfig.aireciudadano_device_name, 25);
#endif
  char Ptime[5];
  itoa(eepromConfig.PublicTime, Ptime, 10);
  WiFiManagerParameter custom_public_time("Ptime", "Set Publication Time in minutes:", Ptime, 4);
  WiFiManagerParameter custom_mqtt_html("<p></p>"); // only custom html
  WiFiManagerParameter custom_mqtt_server("Server", "MQTT server:", eepromConfig.MQTT_server, 32);
  char port[6];
  itoa(eepromConfig.MQTT_port, port, 10);
  WiFiManagerParameter custom_mqtt_port("Port", "MQTT port:", port, 6);
  WiFiManagerParameter custom_sensor_html("<p></p>"); // only custom html
  WiFiManagerParameter custom_sensor_latitude("Latitude", "Latitude sensor (5-4 dec digits are enough)", eepromConfig.sensor_lat, 10);
  WiFiManagerParameter custom_sensor_longitude("Longitude", "Longitude sensor", eepromConfig.sensor_lon, 10);
  WiFiManagerParameter custom_sensorPM_type;
  WiFiManagerParameter custom_sensorHYT_type;
  WiFiManagerParameter custom_display_type;
  WiFiManagerParameter custom_board_type;
  WiFiManagerParameter custom_outin_type;
  WiFiManagerParameter custom_endhtml("<p></p>"); // only custom html

  // Sensor PM menu

  if (eepromConfig.ConfigValues[8] == '0')
  {
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM model:</label><br/><input type='radio' name='customSenPM' value='0' checked> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS raw NOT recommended<br><input type='radio' name='customSenPM' value='4'> Plantower PMS adjust RECOMMENDED";
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[8] == '1')
  {
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM model:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1' checked> Sensirion SPS30<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS raw NOT recommended<br><input type='radio' name='customSenPM' value='4'> Plantower PMS adjust RECOMMENDED";
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[8] == '2')
  {
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM model:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30<br><input type='radio' name='customSenPM' value='2' checked> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS raw NOT recommended<br><input type='radio' name='customSenPM' value='4'> Plantower PMS adjust RECOMMENDED";
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[8] == '3')
  {
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM model:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3' checked> Plantower PMS raw NOT recommended<br><input type='radio' name='customSenPM' value='4'> Plantower PMS adjust RECOMMENDED";
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[8] == '4')
  {
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM model:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS raw NOT recommended<br><input type='radio' name='customSenPM' value='4' checked> Plantower PMS adjust RECOMMENDED";
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }

  // Sensor HYT menu

  if (eepromConfig.ConfigValues[7] == '0')
  {
    const char *custom_senHYT_str = "<br/><br/><label for='customSenHYT'>Sensor HYT type:</label><br/><input type='radio' name='customSenHYT' value='0' checked> None<br><input type='radio' name='customSenHYT' value='1'> Sensirion SHT31<br><input type='radio' name='customSenHYT' value='2'> AM2320";
    new (&custom_sensorHYT_type) WiFiManagerParameter(custom_senHYT_str);
  }
  else if (eepromConfig.ConfigValues[7] == '1')
  {
    const char *custom_senHYT_str = "<br/><br/><label for='customSenHYT'>Sensor HYT type:</label><br/><input type='radio' name='customSenHYT' value='0'> None<br><input type='radio' name='customSenHYT' value='1' checked> Sensirion SHT31<br><input type='radio' name='customSenHYT' value='2'> AM2320";
    new (&custom_sensorHYT_type) WiFiManagerParameter(custom_senHYT_str);
  }
  else if (eepromConfig.ConfigValues[7] == '2')
  {
    const char *custom_senHYT_str = "<br/><br/><label for='customSenHYT'>Sensor HYT type:</label><br/><input type='radio' name='customSenHYT' value='0'> None<br><input type='radio' name='customSenHYT' value='1'> Sensirion SHT31<br><input type='radio' name='customSenHYT' value='2' checked> AM2320";
    new (&custom_sensorHYT_type) WiFiManagerParameter(custom_senHYT_str);
  }

  // Sensor Display menu

  if (eepromConfig.ConfigValues[6] == '0')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display model:</label><br/><input type='radio' name='customDisplay' value='0' checked> Without display<br><input type='radio' name='customDisplay' value='1'> TTGO T-Display<br><input type='radio' name='customDisplay' value='2'> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3'> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }
  else if (eepromConfig.ConfigValues[6] == '1')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display model:</label><br/><input type='radio' name='customDisplay' value='0'> Without display<br><input type='radio' name='customDisplay' value='1' checked> TTGO T-Display<br><input type='radio' name='customDisplay' value='2'> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3'> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }
  else if (eepromConfig.ConfigValues[6] == '2')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display model:</label><br/><input type='radio' name='customDisplay' value='0'> Without display<br><input type='radio' name='customDisplay' value='1'> TTGO T-Display<br><input type='radio' name='customDisplay' value='2' checked> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3'> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }
  else if (eepromConfig.ConfigValues[6] == '3')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display model:</label><br/><input type='radio' name='customDisplay' value='0'> Without display<br><input type='radio' name='customDisplay' value='1'> TTGO T-Display<br><input type='radio' name='customDisplay' value='2'> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3' checked> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }

  // Sensor Board menu

  if (eepromConfig.ConfigValues[5] == '0')
  {
    const char *custom_board_str = "<br/><br/><label for='customBoard'>Board model:</label><br/><input type='radio' name='customBoard' value='0' checked> Normal (internal antenna)<br><input type='radio' name='customBoard' value='1'> Board for external antenna";
    new (&custom_board_type) WiFiManagerParameter(custom_board_str);
  }
  else if (eepromConfig.ConfigValues[5] == '1')
  {
    const char *custom_board_str = "<br/><br/><label for='customBoard'>Board model:</label><br/><input type='radio' name='customBoard' value='0'> Normal (internal antenna)<br><input type='radio' name='customBoard' value='1' checked> Board for external antenna";
    new (&custom_board_type) WiFiManagerParameter(custom_board_str);
  }

  // Sensor Location menu

  if (eepromConfig.ConfigValues[4] == '0')
  {
    const char *custom_outin_str = "<br/><br/><label for='customOutIn'>IMPORTANT:</label><br/><input type='radio' name='customOutIn' value='0' checked> Outdoors - sensor measures outdoors air<br><input type='radio' name='customOutIn' value='1'> Indoors - sensor measures indoors air";
    new (&custom_outin_type) WiFiManagerParameter(custom_outin_str);
  }
  else if (eepromConfig.ConfigValues[4] == '1')
  {
    const char *custom_outin_str = "<br/><br/><label for='customOutIn'>IMPORTANT:</label><br/><input type='radio' name='customOutIn' value='0'> Outdoors - sensor measures outdoors air<br><input type='radio' name='customOutIn' value='1' checked> Indoors - sensor measures indoors air";
    new (&custom_outin_type) WiFiManagerParameter(custom_outin_str);
  }

  // Add parameters

#if WPA2
  wifiManager.addParameter(&custom_wifi_html);
  wifiManager.addParameter(&custom_wifi_user);
  wifiManager.addParameter(&custom_wifi_password);
  wifiManager.addParameter(&custom_wifi_html2);
#endif

  wifiManager.addParameter(&custom_id_name);
  wifiManager.addParameter(&custom_public_time);
  wifiManager.addParameter(&custom_mqtt_html);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_sensor_html);
  wifiManager.addParameter(&custom_sensor_latitude);
  wifiManager.addParameter(&custom_sensor_longitude);
  wifiManager.addParameter(&custom_sensorPM_type);
  wifiManager.addParameter(&custom_sensorHYT_type);
  wifiManager.addParameter(&custom_display_type);
  wifiManager.addParameter(&custom_board_type);
  wifiManager.addParameter(&custom_outin_type);
  wifiManager.addParameter(&custom_endhtml);

  wifiManager.setSaveParamsCallback(saveParamCallback);

  // sets timeout in seconds until configuration portal gets turned off.
  // If not specified device will remain in configuration mode until
  // switched off via webserver or device is restarted.

  // #if ESP8266
  //   wifiManager.setSaveConnectTimeout(connectiontimeout);
  //   Serial.print(F("setSaveConnectTimeout "));
  //   Serial.println(connectiontimeout);
  // #endif

  // wifiManager.setScanDispPerc(true);

  wifiManager.setConfigPortalTimeout(captiveportaltime);

  const char *menu[] = {"wifi", "wifinoscan", "info", "exit", "sep", "update"};
  wifiManager.setMenu(menu, 6);

  // it starts an access point
  // and goes into a blocking loop awaiting configuration
  // wifiManager.resetSettings(); // reset previous configurations
  bool res = wifiManager.startConfigPortal(wifiAP.c_str());
  if (!res)
  {
    Serial.println(F("Not able to start captive portal"));
  }
  else
  {
    // if you get here you have connected to the WiFi
    Serial.println(F("Captive portal operative"));
  }

  // Save parameters to EEPROM only if any of them changed
  bool write_eeprom = false;

#if WPA2
  if (eepromConfig.wifi_user != custom_wifi_user.getValue())
  {
    strncpy(eepromConfig.wifi_user, custom_wifi_user.getValue(), sizeof(eepromConfig.wifi_user));
    eepromConfig.wifi_user[sizeof(eepromConfig.wifi_user) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("Wifi user write_eeprom = true"));
    Serial.print(F("WiFi user: "));
    Serial.println(eepromConfig.wifi_user);
  }
  if (eepromConfig.wifi_password != custom_wifi_password.getValue())
  {
    strncpy(eepromConfig.wifi_password, custom_wifi_password.getValue(), sizeof(eepromConfig.wifi_password));
    eepromConfig.wifi_password[sizeof(eepromConfig.wifi_password) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("Wifi pass write_eeprom = true"));
    Serial.print(F("WiFi password: "));
    Serial.println(eepromConfig.wifi_password);
  }
#endif

  if (eepromConfig.aireciudadano_device_name != custom_id_name.getValue())
  {
    strncpy(eepromConfig.aireciudadano_device_name, custom_id_name.getValue(), sizeof(eepromConfig.aireciudadano_device_name));
    eepromConfig.aireciudadano_device_name[sizeof(eepromConfig.aireciudadano_device_name) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("Devname write_eeprom = true"));
    Serial.print(F("Device name (captive portal): "));
    Serial.println(eepromConfig.aireciudadano_device_name);
  }

  if (eepromConfig.PublicTime != atoi(custom_public_time.getValue()))
  {
    eepromConfig.PublicTime = atoi(custom_public_time.getValue());
    write_eeprom = true;
    Serial.println(F("PublicTime write_eeprom = true"));
    Serial.print(F("Publication time: "));
    Serial.println(eepromConfig.PublicTime);
  }

  if (eepromConfig.MQTT_server != custom_mqtt_server.getValue())
  {
    strncpy(eepromConfig.MQTT_server, custom_mqtt_server.getValue(), sizeof(eepromConfig.MQTT_server));
    eepromConfig.MQTT_server[sizeof(eepromConfig.MQTT_server) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("MQTT server write_eeprom = true"));
    Serial.print(F("MQTT server: "));
    Serial.println(eepromConfig.MQTT_server);
  }

  if (eepromConfig.MQTT_port != atoi(custom_mqtt_port.getValue()))
  {
    eepromConfig.MQTT_port = atoi(custom_mqtt_port.getValue());
    write_eeprom = true;
    Serial.println(F("MQTT port write_eeprom = true"));
    Serial.print(F("MQTT port: "));
    Serial.println(eepromConfig.MQTT_port);
  }

  if (eepromConfig.sensor_lat != custom_sensor_latitude.getValue())
  {
    strncpy(eepromConfig.sensor_lat, custom_sensor_latitude.getValue(), sizeof(eepromConfig.sensor_lat));
    eepromConfig.sensor_lat[sizeof(eepromConfig.sensor_lat) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("Lat write_eeprom = true"));
    Serial.print(F("Sensor Latitude: "));
    Serial.println(eepromConfig.sensor_lat);
    latitudef = atof(eepromConfig.sensor_lat); // Cambiar de string a float
  }

  if (eepromConfig.sensor_lon != custom_sensor_longitude.getValue())
  {
    strncpy(eepromConfig.sensor_lon, custom_sensor_longitude.getValue(), sizeof(eepromConfig.sensor_lon));
    eepromConfig.sensor_lon[sizeof(eepromConfig.sensor_lon) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("Lon write_eeprom = true"));
    Serial.print(F("Sensor Longitude: "));
    Serial.println(eepromConfig.sensor_lon);
    longitudef = atof(eepromConfig.sensor_lon); // Cambiar de string a float
  }

  CustomValTotalString[9] = {0};
  sprintf(CustomValTotalString, "%9d", CustomValtotal);
  if (CustomValTotalString[0] == ' ')
    CustomValTotalString[0] = '0';
  if (CustomValTotalString[1] == ' ')
    CustomValTotalString[1] = '0';
  if (CustomValTotalString[2] == ' ')
    CustomValTotalString[2] = '0';
  if (CustomValTotalString[3] == ' ')
    CustomValTotalString[3] = '0';
  if (CustomValTotalString[4] == ' ')
    CustomValTotalString[4] = '0';
  if (CustomValTotalString[5] == ' ')
    CustomValTotalString[5] = '0';
  if (CustomValTotalString[6] == ' ')
    CustomValTotalString[6] = '0';
  if (CustomValTotalString[7] == ' ')
    CustomValTotalString[7] = '0';
  if (CustomValTotalString[8] == ' ')
    CustomValTotalString[8] = '0';

  Serial.print(F("CustomValTotalString: "));
  Serial.println(CustomValTotalString);

  if (CustomValtotal == 0)
  {
    Serial.println(F("No configuration sensor values ​​chosen, no changes will be stored"));
  }
  else
  {
    if (eepromConfig.ConfigValues != CustomValTotalString)
    {
      strncpy(eepromConfig.ConfigValues, CustomValTotalString, sizeof(eepromConfig.ConfigValues));
      eepromConfig.ConfigValues[sizeof(eepromConfig.ConfigValues) - 1] = '\0';
      write_eeprom = true;
      Serial.println(F("CustomVal write_eeprom = true"));
      Serial.print(F("Configuration Values: "));
      Serial.println(eepromConfig.ConfigValues);
    }
  }

  if (write_eeprom)
  {
    Write_EEPROM();
    Serial.println(F("write_eeprom = true Final"));
    ESP.restart();
  }

  InCaptivePortal = false;
}

String getParam(String name)
{
  // read parameter from server, for custom hmtl input
  String value;

  if (wifiManager.server->hasArg(name))
  {
    value = wifiManager.server->arg(name);
  }
  CustomValue = atoi(value.c_str());
  return value;
}

void Init_MQTT()
{ // MQTT Init function
  Serial.print(F("Attempting to connect to the MQTT broker "));
  Serial.print(eepromConfig.MQTT_server);
  Serial.print(F(":"));
  Serial.println(eepromConfig.MQTT_port);

  // Attempt to connect to MQTT broker
  MQTT_client.setBufferSize(512); // to receive messages up to 512 bytes length (default is 256)
  Serial.println(F("Paso1"));
  MQTT_client.setServer(eepromConfig.MQTT_server, eepromConfig.MQTT_port);
  Serial.println(F("Paso2"));
  MQTT_client.setCallback(Receive_Message_Cloud_App_MQTT);
  Serial.println(F("Paso3"));
  MQTT_client.connect(aireciudadano_device_id.c_str());
  Serial.println(F("Paso4"));

  if (!MQTT_client.connected())
  {
    err_MQTT = true;
    MQTT_Reconnect();
  }
  else
  {
    err_MQTT = false;
    lastReconnectAttempt = 0;
    // Once connected resubscribe
    MQTT_client.subscribe(MQTT_receive_topic.c_str());
    Serial.print(F("MQTT connected - Receive topic: "));
    Serial.println(MQTT_receive_topic);
  }
}

void MQTT_Reconnect()
{ // MQTT reconnect function
  // Try to reconnect only if it has been more than 5 sec since last attemp
  unsigned long now = millis();
  if (now - lastReconnectAttempt > 5000)
  {
    lastReconnectAttempt = now;
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (MQTT_client.connect(aireciudadano_device_id.c_str()))
    {
      err_MQTT = false;
      Serial.println(F("MQTT connected"));
      lastReconnectAttempt = 0;
      // Once connected resubscribe
      MQTT_client.subscribe(MQTT_receive_topic.c_str());
      Serial.print(F("MQTT connected - Receive topic: "));
      Serial.println(MQTT_receive_topic);
    }
    else
    {
      err_MQTT = true;
      Serial.print(F("failed, rc="));
      Serial.print(MQTT_client.state());
      Serial.println(F(" try again in 5 seconds"));
    }
    Serial.println(F("Paso5"));
  }
}

void Send_Message_Cloud_App_MQTT()
{ // Send measurements to the cloud application by MQTT
  // Print info
  float pm25f;
  float pm25fori;
  int8_t RSSI;
  int8_t inout;

  Serial.print(F("Sending MQTT message to the send topic: "));
  Serial.println(MQTT_send_topic);
  ///// DEBUG Samples
  //  Serial.println(F(PM25_accumulated));
  //  Serial.println(F(PM25_samples));
  pm25f = PM25_accumulated / PM25_samples;
  pm25int = round(pm25f);
  pm25fori = PM25_accumulated_ori / PM25_samples;
  pm25intori = round(pm25fori);
  //  Serial.println(F(pm25int));
  //  Serial.println(F(pm25intori));
  ///// END DEBUG Samples
  ReadHyT();

  RSSI = WiFi.RSSI();

  Serial.print(F("Signal strength (RSSI):"));
  Serial.print(RSSI);
  Serial.println(F(" dBm"));

  if (AmbInOutdoors)
    inout = 1;
  else
    inout = 0;

  if (SEN5Xsen == true)
  {
    uint8_t voc;
    uint8_t nox;

    if (isnan(ambientHumidity))
    {
      if (humi == 0)
        humi = 0;
    }
    else
      humi = round(ambientHumidity);
    if (isnan(ambientTemperature))
    {
      if (temp == 0)
        temp = 0;
    }
    else
      temp = round(ambientTemperature);
    if (isnan(vocIndex))
      voc = 0;
    else
      voc = round(vocIndex);
    if (isnan(noxIndex))
      nox = 0;
    else
      nox = round(noxIndex);
    sprintf(MQTT_message, "{id: %s, PM25: %d, VOC: %d, NOx: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, voc, nox, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);
    // sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"VOC\": %d, \"NOx\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, voc, nox, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf
  }
  else
  {
    if (AdjPMS == true)
      sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);
    // sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"PM25raw\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf

    else
      sprintf(MQTT_message, "{id: %s, PM25: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);
    // sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf
  }
  Serial.print(MQTT_message);
  Serial.println();

#if ESP8285
  digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
  delay(750);                // wait for a 750 msecond
  digitalWrite(LEDPIN, HIGH);
#endif

  if (OLED66 == true || OLED96 == true || TDisplay == true)
    FlagDATAicon = true;

  // send message, the Print interface can be used to set the message contents
  MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);
}

void Receive_Message_Cloud_App_MQTT(char *topic, byte *payload, unsigned int length)
{                               // callback function to receive configuration messages from the cloud application by MQTT
  boolean write_eeprom = false; // to track if writing the eeprom is required
  memcpy(received_payload, payload, length);
  Serial.print(F("Message arrived: "));
  Serial.println(received_payload);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(jsonBuffer, received_payload);

  // Test if parsing succeeds.
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Update name
  if ((jsonBuffer["name"]) && (eepromConfig.aireciudadano_device_name != jsonBuffer["name"]))
  {
    strncpy(eepromConfig.aireciudadano_device_name, jsonBuffer["name"].as<const char *>(), sizeof(eepromConfig.aireciudadano_device_name));
    eepromConfig.aireciudadano_device_name[sizeof(eepromConfig.aireciudadano_device_name) - 1] = '\0';
    Serial.print(F("AireCiudadano custom name (json buffer): "));
    Serial.println(eepromConfig.aireciudadano_device_name);
    write_eeprom = true;
  }

  // Check MQTT server
  if ((jsonBuffer["MQTT_server"]) && (eepromConfig.MQTT_server != jsonBuffer["MQTT_server"]))
  {
    strncpy(eepromConfig.MQTT_server, jsonBuffer["MQTT_server"], sizeof(eepromConfig.MQTT_server));
    eepromConfig.MQTT_server[sizeof(eepromConfig.MQTT_server) - 1] = '\0';
    Serial.print(F("MQTT Server: "));
    Serial.println(eepromConfig.MQTT_server);
    write_eeprom = true;

    // Attempt to connect to MQTT broker
    if (!err_wifi)
    {
      Init_MQTT();
    }
  }

  // Check MQTT port
  if ((jsonBuffer["MQTT_port"]) && (eepromConfig.MQTT_port != int(jsonBuffer["MQTT_port"])))
  {
    eepromConfig.MQTT_port = int(jsonBuffer["MQTT_port"]);
    // strncpy(eepromConfig.MQTT_port, jsonBuffer["MQTT_port"], sizeof(eepromConfig.MQTT_port));
    // eepromConfig.MQTT_port[sizeof(eepromConfig.MQTT_port) - 1] = '\0';
    Serial.print(F("MQTT Port: "));
    Serial.println(eepromConfig.MQTT_port);
    write_eeprom = true;

    // Attempt to connect to MQTT broker
    if (!err_wifi)
    {
      Init_MQTT();
    }
  }

  // print info
  Serial.println(F("MQTT update - message processed"));
  Print_Config();

  // If factory reset has been enabled, just do it
  if ((jsonBuffer["factory_reset"]) && (jsonBuffer["factory_reset"] == "ON"))
  {
    Wipe_EEPROM(); // Wipe EEPROM
    ESP.restart();
  }

  // If reboot, just do it, without cleaning the EEPROM
  if ((jsonBuffer["reboot"]) && (jsonBuffer["reboot"] == "ON"))
  {
    ESP.restart();
  }

  // save the new values if the flag was set
  if (write_eeprom)
  {
    Write_EEPROM();
  }

  // if update flag has been enabled, update to latest bin
  // It has to be the last option, to allow to save EEPROM if required
  if (((jsonBuffer["update"]) && (jsonBuffer["update"] == "ON")))
  {
    // boolean result = EEPROM.wipe();
    // if (result) {
    //   Serial.println(F("All EEPROM data wiped"));
    // } else {
    //   Serial.println(F("EEPROM data could not be wiped from flash store"));
    // }

    // Update firmware to latest bin
    Serial.println(F("Update firmware to latest bin"));
    Firmware_Update();
  }
}

void Firmware_Update()
{

#if !ESP8266

  Serial.println(F("### FIRMWARE UPDATE ###"));

  // For remote firmware update
  WiFiClientSecure UpdateClient;
  UpdateClient.setInsecure();

  // Reading data over SSL may be slow, use an adequate timeout
  UpdateClient.setTimeout(30); // timeout argument is defined in seconds for setTimeout
  Serial.println(F("ACTUALIZACION EN CURSO"));

#if Tdisplaydisp

  if (TDisplay == true)
  {
    // Update display
    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("ACTUALIZACION EN CURSO", tft.width() / 2, tft.height() / 2);
  }

#endif

  // t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/src/anaire.PiCO2/anaire.PiCO2.ino.esp32.bin");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/Anaire.PiCO2/anaire.PiCO2/anaire.PiCO2.ino.esp32.bin");

  switch (ret)
  {

  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());

#if Tdisplaydisp

    if (TDisplay == true)
    {
      tft.fillScreen(TFT_ORANGE);
      tft.drawString("ACTUALIZACION FALLIDA", tft.width() / 2, tft.height() / 2);
    }

#endif

    delay(1000);
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
    break;

  case HTTP_UPDATE_OK:
    Serial.println(F("HTTP_UPDATE_OK"));

#if Tdisplaydisp

    if (TDisplay == true)
    {
      tft.fillScreen(TFT_ORANGE);
      tft.drawString("ACTUALIZACION COMPLETA", tft.width() / 2, tft.height() / 2);
    }

#endif

    delay(1000);
    break;
  }

#else

  // For remote firmware update
  BearSSL::WiFiClientSecure UpdateClient;
  int freeheap = ESP.getFreeHeap();

  Serial.println(F("### FIRMWARE UPGRADE ###"));

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
  if (mfln)
  {
    UpdateClient.setBufferSizes(512, 512);
  }
  UpdateClient.connect("raw.githubusercontent.com", 443);
  if (UpdateClient.connected())
  {
    Serial.printf("MFLN status: %s\n", UpdateClient.getMFLNStatus() ? "true" : "false");
    Serial.printf("Memory used: %d\n", freeheap - ESP.getFreeHeap());
    freeheap -= ESP.getFreeHeap();
  }
  else
  {
    Serial.printf("Unable to connect\n");
  }

  // Run http update
  // t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin");

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
    break;

  case HTTP_UPDATE_OK:
    Serial.println(F("HTTP_UPDATE_OK"));
    break;
  }

#endif
}

#if ESP8266

void update_started()
{
  Serial.println(F("CALLBACK:  HTTP update process started"));
  updating = true;
}

void update_finished()
{
  Serial.println(F("CALLBACK:  HTTP update process finished"));
  Serial.println(F("### FIRMWARE UPGRADE COMPLETED - REBOOT ###"));
  updating = false;
}

void update_progress(int cur, int total)
{
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err)
{
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
  updating = false;
}

#endif

#endif

void saveParamCallback()
{
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("Value customSenPM = " + getParam("customSenPM"));
  CustomValtotal = CustomValue;
  Serial.println("Value cutomSenHYT = " + getParam("customSenHYT"));
  CustomValtotal = CustomValtotal + (CustomValue * 10);
  Serial.println("Value customDisplay = " + getParam("customDisplay"));
  CustomValtotal = CustomValtotal + (CustomValue * 100);
  Serial.println("Value customBoard = " + getParam("customBoard"));
  CustomValtotal = CustomValtotal + (CustomValue * 1000);
  Serial.println("Value customOutIn = " + getParam("customOutIn"));
  CustomValtotal = CustomValtotal + (CustomValue * 10000);
  Serial.print("CustomValtotal: ");
  Serial.println(CustomValtotal);
}

void Setup_Sensor()
{ // Identify and initialize PM25, temperature and humidity sensor

  // Test PM2.5 SPS30

#if Wifi
  if (SPS30sen == true)
  {
#endif
    Serial.println(F("Test Sensirion SPS30 sensor"));

#if ESP8266
    Wire.begin();
#else
  Wire.begin(Sensor_SDA_pin, Sensor_SCL_pin);
#endif

    sps30.EnableDebugging(DEBUG);
    // Begin communication channel
    SP30_COMMS.begin();
    if (sps30.begin(&SP30_COMMS) == false)
    {
      Errorloop((char *)"Could not set I2C communication channel.", 0);
    }
    // check for SPS30 connection
    if (!sps30.probe())
      Errorloop((char *)"could not probe / connect with SPS30.", 0);
    else
    {
      Serial.println(F("Detected I2C Sensirion Sensor"));
      GetDeviceInfo();
    }
    if (SPS30sen == true)
    {
      // start measurement
      if (sps30.start())
        Serial.println(F("Measurement started"));
      else
        Errorloop((char *)"Could NOT start measurement", 0);
    }
#if Wifi
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Test PM2.5 SEN5X

  if (SEN5Xsen == true)

  {
#endif
#if (Bluetooth || SDyRTC)
    if (SPS30sen == false)
    {
#endif
      Serial.println(F("Test Sensirion SEN5X sensor"));

#if ESP8266
      Wire.begin();
#else
  Wire.begin(Sensor_SDA_pin, Sensor_SCL_pin);
#endif

      delay(10);
      sen5x.begin(Wire);
      delay(10);

      uint16_t error;
      char errorMessage[256];
      error = sen5x.deviceReset();
      if (error)
      {
        Serial.print(F("Error trying to execute deviceReset(): "));
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
      }
      else
      {
        // Print SEN55 module information if i2c buffers are large enough
        Serial.println(F("SEN5X sensor found!"));
        SEN5Xsen = true;
        printSerialNumber();
        printModuleVersions();

        // Start Measurement
        error = sen5x.startMeasurement();
        if (error)
        {
          Serial.print(F("Error trying to execute startMeasurement(): "));
          errorToString(error, errorMessage, 256);
          Serial.println(errorMessage);
          // ESP.restart();
        }
        else
          Serial.println(F("SEN5X measurement OK"));
      }
#if (Bluetooth || SDyRTC)
    }
#else
}

///////////////////////////////////////////////////////////////////////////////////////////

// PMS7003 PMSA003
if (PMSsen == true)
{
#endif
    Serial.println(F("Test Plantower Sensor"));

#if !ESP8266

#if !TTGO_TQ
    Serial1.begin(PMS::BAUD_RATE, SERIAL_8N1, PMS_TX, PMS_RX);
#else
    Serial2.begin(PMS::BAUD_RATE, SERIAL_8N1, PMS_TX, PMS_RX);
#endif

#else
  pmsSerial.begin(9600); // Software serial begin for PMS sensor
  Serial.println(F("Test5"));
#endif

    delay(1000);
  Serial.println(F("Test6"));

    if (pms.readUntil(data))
    {
      Serial.println(F("Plantower sensor found!"));
      PMSsen = true;
#if ESP8285
      digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
      delay(750);                // wait for a 750 msecond
      digitalWrite(LEDPIN, HIGH);
#endif
#if (Bluetooth || SDyRTC)
      AdjPMS = true; // Por defecto se deja con ajuste, REVISAR!!!!!!
#endif
    }
    else
    {
      Serial.println(F("Could not find Plantower sensor!"));
    }

#if Wifi
  }

  if (SHT31sen == true)
  {
#endif
    Serial.print(F("SHT31 test: "));
    if (!sht31.begin(0x44))
    { // Set to 0x45 for alternate i2c addr
      Serial.println(F("none"));
    }
    else
    {
      Serial.println(F("OK"));
      SHT31sen = true;
    }

    Serial.print(F("Heater Enabled State: "));
    if (sht31.isHeaterEnabled())
      Serial.println(F("ENABLED"));
    else
      Serial.println(F("DISABLED"));
#if Wifi
  }

  if (AM2320sen == true)
  {
#endif
    Serial.print(F("AM2320 test: "));
    am2320.begin();
    delay(1);
    humidity = am2320.readHumidity();
    temperature = am2320.readTemperature();
    if (!isnan(humidity))
    {
      Serial.println(F("OK"));
      AM2320sen = true;
    }
    else
      Serial.println(F("none"));
#if Wifi
  }
#endif
}

void Read_Sensor()
{ // Read PM25, temperature and humidity values

  if (SPS30sen == true)
  {
    uint8_t ret, error_cnt = 0;
    struct sps_values val;
    // loop to get data
    do
    {
      ret = sps30.GetValues(&val);
      // data might not have been ready
      if (ret == SPS30_ERR_DATALENGTH)
      {
        if (error_cnt++ > 3)
        {
          ErrtoMess((char *)"Error during reading values: ", ret);
          // return(false);
        }
        delay(1000);
      }
      // if other error
      else if (ret != SPS30_ERR_OK)
      {
        ErrtoMess((char *)"Error during reading values: ", ret);
        // return(false);
      }
    } while (ret != SPS30_ERR_OK);

    PM25_value = val.MassPM2;

    if (!err_sensor)
    {
      // Provide the sensor values for Tools -> Serial Monitor or Serial Plotter
      Serial.print(F("SPS30 PM2.5: "));
      Serial.print(PM25_value);
      Serial.println(F(" ug/m3   "));
    }
  }
  else if (SEN5Xsen == true)
  {
    uint16_t error;
    char errorMessage[256];

    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error)
    {
      Serial.print(F("Error trying to execute readMeasuredValues(): "));
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      delay(10);
      Setup_Sensor();
      Serial.println(F("Reinit I2C"));
      delay(10);
    }
    else
    {
      PM25_value = massConcentrationPm2p5;
      Serial.print(F("SEN5X PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));
      Serial.print(F(" Humi % = "));
      if (isnan(ambientHumidity))
        Serial.print(F(" n/a"));
      else
        Serial.print(ambientHumidity);
      humi = round(ambientHumidity);

      Serial.print(F("   Temp *C = "));
      if (isnan(ambientTemperature))
        Serial.print(F(" n/a"));
      else
        Serial.print(ambientTemperature);
      temp = round(ambientTemperature);
      Serial.print(F("   VocIndex:"));
      if (isnan(vocIndex))
        Serial.print(F(" n/a"));
      else
        Serial.print(vocIndex);
      Serial.print(F("   NoxIndex:"));
      if (isnan(noxIndex))
        Serial.println(F(" n/a"));
      else
        Serial.println(noxIndex);
    }
  }
  else if (PMSsen == true)
  {
    if (pms.readUntil(data))
    {
      PM25_value = data.PM_AE_UG_2_5;
      Serial.print(F("PMS PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));
      if (AdjPMS == true)
      {
        PM25_value_ori = PM25_value;
        // PM25_value = ((562 * PM25_value_ori) / 1000) - 1; // Ecuación de ajuste resultado de 13 intercomparaciones entre PMS7003 y SPS30 por meses
        // PM25_value = ((553 * PM25_value_ori) / 1000) + 1.3; // Segundo ajuste
        PM25_value = ((630 * PM25_value_ori) / 1000) + 1.56; // Tercer ajuste a los que salio en Lima y pruebas aqui
        Serial.print(F("Adjust: "));
        Serial.print(PM25_value);
        Serial.println(F(" ug/m3"));
      }
      else
        Serial.println(F(""));
    }
    else
    {
      Serial.println(F("No data by Plantower sensor!"));
    }
  }
  else
    NoSensor = true;
}

/**
 * @brief : read and display device info
 */

void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;
  // try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK)
  {
    Serial.print(F("Serial number : "));
    if (strlen(buf) > 0)
      Serial.println(buf);
    else
      Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *)"could not get serial number. ", ret);
  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)
  {
    Serial.print(F("Product name  : ")); //     !!!!!!!!!!!!!!!!!!debe compararse con “00080000”

    if (buf[7] == '0')
      if (buf[6] == '0')
        if (buf[5] == '0')
          if (buf[4] == '0')
            if (buf[3] == '8')
            {
              Serial.println(buf);
              Serial.println(F("Detected SPS30"));
              SPS30sen = true;
            }
            else
              NotAvailableSPS30();
          else
            NotAvailableSPS30();
        else
          NotAvailableSPS30();
      else
        NotAvailableSPS30();
    else
      NotAvailableSPS30();
  }
  else
    ErrtoMess((char *)"could not get product name. ", ret);
  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK)
  {
    Serial.println(F("Can not read version info."));
    return;
  }

  if (SPS30sen == true)
  {
    Serial.print(F("Firmware level: "));
    Serial.print(v.major);
    Serial.print(F("."));
    Serial.println(v.minor);

    Serial.print(F("Library level : "));
    Serial.print(v.DRV_major);
    Serial.print(F("."));
    Serial.println(v.DRV_minor);
  }
}

void NotAvailableSPS30()
{
  Serial.println(F("NO SPS30"));
  SPS30sen = false;
}

void Errorloop(char *mess, uint8_t r)
{
  if (r)
    ErrtoMess(mess, r);
  else
    Serial.println(mess);
  Serial.println(F("No SPS30 connected"));
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];
  Serial.print(mess);
  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

void printModuleVersions()
{
  uint16_t error;
  char errorMessage[256];

  unsigned char productName[32];
  uint8_t productNameSize = 32;

  error = sen5x.getProductName(productName, productNameSize);

  if (error)
  {
    Serial.print(F("Error trying to execute getProductName(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print(F("ProductName: "));
    Serial.println((char *)productName);
  }

  bool firmwareDebug;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint8_t hardwareMajor;
  uint8_t hardwareMinor;
  uint8_t protocolMajor;
  uint8_t protocolMinor;

  error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                           hardwareMajor, hardwareMinor, protocolMajor,
                           protocolMinor);
  if (error)
  {
    Serial.print(F("Error trying to execute getVersion(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print(F("Firmware: "));
    Serial.print(firmwareMajor);
    Serial.print(F("."));
    Serial.print(firmwareMinor);
    Serial.print(F(", "));

    Serial.print(F("Hardware: "));
    Serial.print(hardwareMajor);
    Serial.print(F("."));
    Serial.println(hardwareMinor);
  }
}

void printSerialNumber()
{
  uint16_t error;
  char errorMessage[256];
  unsigned char serialNumber[32];
  uint8_t serialNumberSize = 32;

  error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
  if (error)
  {
    Serial.print(F("Error trying to execute getSerialNumber(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print(F("SerialNumber: "));
    Serial.println((char *)serialNumber);
  }
}

///////////////////////////////////////////////////////////////////////////////
void ReadHyT()
{
  /////////  SHT31
  if (SHT31sen == true)
  {
    temperature = 0.0;
    humidity = 0.0;
    humidity = sht31.readHumidity();
    temperature = sht31.readTemperature();

    if (!isnan(humidity))
    { // check if 'is not a number'
      failh = 0;
      Serial.print(F("SHT31 Humi % = "));
      Serial.print(humidity);
      humi = round(humidity);
    }
    else
    {
      Serial.println(F("Failed to read humidity SHT31"));
      if (failh == 5)
      {
        failh = 0;
        sht31.begin(0x44);
      }
      else
        failh = failh + 1;
    }

    if (!isnan(temperature))
    { // check if 'is not a number'
      Serial.print(F("   Temp *C = "));
      Serial.println(temperature);
      temp = round(temperature);
    }
    else
      Serial.println(F("   Failed to read temperature SHT31"));
  }

  // AM2320//
  else if (AM2320sen == true)
  {
    temperature = 0.0;
    humidity = 0.0;
    humidity = am2320.readHumidity();
    temperature = am2320.readTemperature();

    if (!isnan(humidity))
    {
      failh = 0;
      Serial.print(F("AM2320 Humi % = "));
      Serial.print(humidity);
      humi = round(humidity);
    }
    else
    {
      Serial.println(F("   Failed to read humidity AM2320"));
      if (failh == 5)
      {
        failh = 0;
        am2320.begin();
        Serial.println(F("   Reinit AM2320"));
      }
      else
        failh = failh + 1;
    }

    if (!isnan(temperature))
    {
      Serial.print(F("   Temp *C = "));
      Serial.println(temperature);
      temp = round(temperature);
    }
    else
      Serial.println(F("   Failed to read temperature AM2320"));
  }
  else
    Serial.println();
}

///////////////////////////////////////////////////////////////////////////////

void Print_Config()
{ // print AireCiudadano device settings

  Serial.println(F("#######################################"));
  Serial.print(F("Device id: "));
  Serial.println(aireciudadano_device_id);
#if !SDyRTC
  Serial.print(F("AireCiudadano custom name: "));
  Serial.println(eepromConfig.aireciudadano_device_name);
#endif
  Serial.print(F("SW version: "));
  Serial.println(sw_version);
#if Bluetooth
  Serial.print(F("Bluetooth Time: "));
  Serial.println(eepromConfig.BluetoothTime);
#elif SDyRTC
  Serial.print(F("SDyRTC Time: "));
  Serial.println(SDyRTCtime);
#elif Wifi
  Serial.print(F("Publication Time: "));
  Serial.println(eepromConfig.PublicTime);
  Serial.print(F("MQTT server: "));
  Serial.println(eepromConfig.MQTT_server);
  Serial.print(F("MQTT Port: "));
  Serial.println(eepromConfig.MQTT_port);
  Serial.print(F("Sensor latitude: "));
  Serial.println(eepromConfig.sensor_lat);
  Serial.print(F("Sensor longitude: "));
  Serial.println(eepromConfig.sensor_lon);
  Serial.print(F("Configuration values: "));
  Serial.println(eepromConfig.ConfigValues);
#if WPA2
  Serial.print(F("WiFi user for WPA enterprise: "));
  Serial.println(eepromConfig.wifi_user);
  Serial.print(F("WiFi user's password for WPA enterprise: "));
  Serial.println(eepromConfig.wifi_password);
#endif
#endif
  Serial.println(F("#######################################"));
}

#if !ESP8266
void espDelay(int ms)
{ //! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}
#endif

#if (Tdisplaydisp || OLED96display || OLED66display)

#if !ESP8266
#if Tdisplaydisp

void Button_Init()
{ // Manage TTGO T-Display board buttons

  // Buttons design:
  //   - Top button short click: info about the device
  //   - Top button long click: sleep
  //   - Bottom button short click: buttons usage
  //   - Bottom button long click: config device

  // Long clicks: keep pressing more than 2 second
  button_top.setLongClickTime(2000);
  button_bottom.setLongClickTime(2000);
  //  Serial.println(F("Button_Init");

  // Top button short click: show info about the device
  button_top.setClickHandler([](Button2 &b)
                             {
    Serial.println(F("Top button short click"));
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextDatum(TL_DATUM); // top left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("ID " + aireciudadano_device_id, 8, 5);         //!!!Arreglar por nuevo tamaño String
    tft.drawString("SW " + sw_version, 8, 22);
#if Bluetooth
    tft.drawString("Bluetooth ver", 8, 39);
    tft.drawString("Log int: " + String(Bluetooth_loop_time), 8, 56);
#elif SDyRTC
    tft.drawString("SDyRTC ver", 8, 39);
    tft.drawString("Log int: " + String(SDyRTCtime), 8, 56);
#elif Wifi
    tft.drawString("Wifi ver", 8, 39);
    tft.drawString("Pubtime min: " + String(eepromConfig.PublicTime), 8, 56);
    tft.drawString("SSID " + String(WiFi.SSID()), 8, 73);
    tft.drawString("IP " + WiFi.localIP().toString(), 8, 90);
    tft.drawString("MAC " + String(WiFi.macAddress()), 8, 107);
    tft.drawString("RSSI " + String(WiFi.RSSI()), 8, 124);
#endif

    delay(5000); // keep the info in the display for 5s
    Update_Display(); }));

  // Top button long click: toggle acoustic alarm
  button_top.setLongClickDetectedHandler([](Button2 &b)
                                         {
    Serial.println(F("Top button long click"));

    Suspend_Device(); });

  // Bottom button short click: show buttons info
  button_bottom.setClickHandler([](Button2 &b)
                                {
    Serial.println(F("Bottom button short click"));
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextDatum(TL_DATUM); // top left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("Der Corto: Info", 3, 5);
    tft.drawString(" Largo: Dormir", 3, 21);
    tft.drawString("Izq Corto:Menu", 3, 69);
    tft.drawString(" Largo: Tiempo", 3, 85);
    delay(5000);
    Update_Display(); });

  // Bottom button long click: deactivate self calibration and perform sensor forced recalibration
  button_bottom.setLongClickDetectedHandler([](Button2 &b)
                                            {
                                              Serial.println(F("Bottom button long click");

                                              tft.fillScreen(TFT_WHITE);
                                              tft.setTextColor(TFT_RED, TFT_WHITE);
                                              tft.setTextSize(1);
                                              tft.setFreeFont(FF90);
                                              tft.setTextDatum(MC_DATUM);
#if Bluetooth
                                              tft.drawString("Tiempo eval:", tft.width() / 2, tft.height() / 2 - 15);
                                              tft.drawString(String(eepromConfig.BluetoothTime) + " seg", tft.width() / 2, tft.height() / 2 + 15);
                                              delay(1000);

                                              Bluetooth_loop_time = eepromConfig.BluetoothTime;

                                              while (digitalRead(BUTTON_BOTTOM) == false)
                                              {
                                                if (Bluetooth_loop_time == 2)
                                                  Bluetooth_loop_time = 10;
                                                else if (Bluetooth_loop_time == 10)
                                                  Bluetooth_loop_time = 60;
                                                else if (Bluetooth_loop_time == 60)
                                                  Bluetooth_loop_time = 120;
                                                else if (Bluetooth_loop_time == 120)
                                                  Bluetooth_loop_time = 300;
                                                else if (Bluetooth_loop_time == 300)
                                                  Bluetooth_loop_time = 600;
                                                else if (Bluetooth_loop_time == 600)
                                                  Bluetooth_loop_time = 3600;
                                                else if (Bluetooth_loop_time == 3600)
                                                  Bluetooth_loop_time = 10800;
                                                else
                                                  Bluetooth_loop_time = 2;
                                                tft.drawString("                    ", tft.width() / 2, tft.height() / 2 + 15);
                                                tft.drawString(String(Bluetooth_loop_time) + " seg", tft.width() / 2, tft.height() / 2 + 15);
                                                delay(1000);
                                              }
                                              tft.drawString(String(Bluetooth_loop_time) + " seg", tft.width() / 2, tft.height() / 2 + 15);
                                              delay(1000);
                                              FlashBluetoothTime();
#else
                                              tft.drawString("Reiniciando", tft.width() / 2, tft.height() / 2 - 5);
                                              delay(2000);
                                              ESP.restart();
#endif
                                            });
}

void Display_Init()
{ // TTGO T-Display init
  tft.init();
  tft.setRotation(0);
}

void Display_Splash_Screen()
{ // Display AireCiudadano splash screen
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 135, 240, Icono_AireCiudadano);
}

void Update_Display()
{ // Update display

  tft.setTextDatum(TL_DATUM); // top left

  // Set screen and text colours based on PM25 value

  displayAverage(pm25int);

  if (FlagDATAicon == true)
  {
    if (pm25int < 57)
      tft.drawXBitmap(5, 194, Icono_data_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(5, 194, Icono_data_on_BIG, 19, 19, TFT_WHITE);
    FlagDATAicon = false;
  }
  if (toggleLive)
  {
#if Bluetooth
    if (pm25int < 57)
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_WHITE);
#endif
  }
  toggleLive = !toggleLive;
}

#endif
#endif

#if (OLED96display || OLED66display)

void UpdateOLED()
{
  //  Serial.println(F("Sensor Read Update OLED");
  pageStart();
  displaySensorAverage(pm25int);
#if Wifi

  // #if !ESP8266
  displaySensorData(round(PM25_value), humi, temp, WiFi.RSSI());
  // #else
  //   displaySensorData(round(PM25_value), humi, temp, WiFi.RSSI());
  // #endif

  if (FlagDATAicon == true)
  {
    u8g2.drawBitmap(dw - 25, dh - 7, 1, 8, Icono_data_on);
    FlagDATAicon = false;
  }
#else
  displaySensorData(round(PM25_value), humi, temp, 0);
  TimeConfig();
#endif
  if (toggleLive)
#if Bluetooth
    u8g2.drawBitmap(dw - 19, dh - 8, 1, 8, Icono_bt_on);
#else
    u8g2.drawBitmap(dw - 15, dh - 7, 1, 8, Icono_sensor_live);
#endif
  toggleLive = !toggleLive;
  pageEnd();
}
#endif
#endif

#if Bluetooth
void TimeConfig()
{
  if (digitalRead(BUTTON_BOTTOM) == false)
  {
    delay(10);
    if (digitalRead(BUTTON_BOTTOM) == false)
    {

#if (OLED96display || OLED66display)

      pageStart();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, dh / 2 - 7);
      u8g2.print("Tiempo eval:");
      u8g2.setCursor(10, dh / 2 + 7);
      u8g2.print(String(eepromConfig.BluetoothTime) + " seg");
      pageEnd();

#endif

      delay(1000);

      Bluetooth_loop_time = eepromConfig.BluetoothTime;

      while (digitalRead(BUTTON_BOTTOM) == false)
      {
        if (Bluetooth_loop_time == 2)
          Bluetooth_loop_time = 10;
        else if (Bluetooth_loop_time == 10)
          Bluetooth_loop_time = 60;
        else if (Bluetooth_loop_time == 60)
          Bluetooth_loop_time = 120;
        else if (Bluetooth_loop_time == 120)
          Bluetooth_loop_time = 300;
        else if (Bluetooth_loop_time == 300)
          Bluetooth_loop_time = 600;
        else if (Bluetooth_loop_time == 600)
          Bluetooth_loop_time = 3600;
        else if (Bluetooth_loop_time == 3600)
          Bluetooth_loop_time = 10800;
        else
          Bluetooth_loop_time = 2;

#if (OLED96display || OLED66display)

        pageStart();
        u8g2.setCursor(0, dh / 2 - 7);
        u8g2.print("Tiempo eval");
        u8g2.setCursor(8, dh / 2 + 7);
        u8g2.print(String(Bluetooth_loop_time) + " seg");
        pageEnd();

#endif
        Serial.print(F("Tiempo evaluacion: "));
        Serial.print(F(Bluetooth_loop_time));
        Serial.println(F(" seg"));
        delay(1000);
      }
      FlashBluetoothTime();
    }
  }
}

void FlashBluetoothTime()
{
  gadgetBle.setSampleIntervalMs(Bluetooth_loop_time * 1000); // Rutina para configurar el tiempo de muestreo del sensor y la app

  if (eepromConfig.BluetoothTime != Bluetooth_loop_time)
  {
    eepromConfig.BluetoothTime = Bluetooth_loop_time;
    Serial.print(F("Bluetooth time: "));
    Serial.println(F(eepromConfig.BluetoothTime));
    Write_EEPROM();
  }
}

#endif

void Get_AireCiudadano_DeviceId()
{ // Get TTGO T-Display info and fill up aireciudadano_device_id with last 6 digits (in HEX) of WiFi mac address or Custom_Name + 6 digits
  //  uint32_t chipId = 0;
  char aireciudadano_device_id_endframe[10];

#if !ESP8266

  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  chipIdHEX = String(chipId, HEX);
  strncpy(aireciudadano_device_id_endframe, chipIdHEX.c_str(), sizeof(aireciudadano_device_id_endframe));
#if Wifi
  Aireciudadano_Characteristics();
#endif
  Serial.printf("ESP32 Chip model = %s Rev %d.\t", ESP.getChipModel(), ESP.getChipRevision()));
  Serial.printf("This chip has %d cores and %dMB Flash.\n", ESP.getChipCores(), ESP.getFlashChipSize() / (1024 * 1024)));

#else

  chipId = ESP.getChipId();
  chipIdHEX = String(ESP.getChipId(), HEX);
  strncpy(aireciudadano_device_id_endframe, chipIdHEX.c_str(), sizeof(aireciudadano_device_id_endframe));
#if Wifi
  Aireciudadano_Characteristics();
#endif

#endif

  Serial.print(F("AireCiudadano Device ID: "));
  if (String(aireciudadano_device_id).isEmpty())
    aireciudadano_device_id = String("AireCiudadano_") + aireciudadano_device_id_endframe;
  else
  {
#if !SDyRTC
    aireciudadano_device_id = String(eepromConfig.aireciudadano_device_name) + "_" + aireciudadano_device_id_endframe;
    // aireciudadano_device_id = String(eepromConfig.aireciudadano_device_name) + aireciudadano_device_id_endframe;
#endif
  }
  Serial.println(aireciudadano_device_id);
}

#if Wifi

void Aireciudadano_Characteristics()
{
  Serial.print(F("eepromConfig.ConfigValues: "));
  Serial.println(eepromConfig.ConfigValues);
  Serial.print(F("eepromConfig.ConfigValues[4]: "));
  Serial.println(eepromConfig.ConfigValues[4]);
  if (eepromConfig.ConfigValues[4] == '0')
  {
    AmbInOutdoors = false;
    Serial.println(F("Outdoors"));
  }
  else
  {
    AmbInOutdoors = true;
    Serial.println(F("Indoors"));
  }

  Serial.print(F("eepromConfig.ConfigValues[5]: "));
  Serial.println(eepromConfig.ConfigValues[5]);
  if (eepromConfig.ConfigValues[5] == '0')
  {
    ExtAnt = false;
    Serial.println(F("Normal board"));
  }
  else
  {
    ExtAnt = true;
    Serial.println(F("Board with externa antenna"));
  }
  Serial.print(F("eepromConfig.ConfigValues[6]: "));
  Serial.println(eepromConfig.ConfigValues[6]);

  TDisplay = false;
  OLED66 = false;
  OLED96 = false;
  if (eepromConfig.ConfigValues[6] == '0')
    Serial.println(F("None Display"));
  else if (eepromConfig.ConfigValues[6] == '1')
  {
    TDisplay = true;
    Serial.println(F("TTGO TDisplay board"));
  }
  else if (eepromConfig.ConfigValues[6] == '2')
  {
    OLED96 = true;
    Serial.println(F("OLED 0.96 inch display 128x64"));
  }
  else if (eepromConfig.ConfigValues[6] == '3')
  {
    OLED66 = true;
    Serial.println(F("OLED 0.66 inch display 64x48"));
  }

  Serial.print(F("eepromConfig.ConfigValues[7]: "));
  Serial.println(eepromConfig.ConfigValues[7]);
  SHT31sen = false;
  AM2320sen = false;
  if (eepromConfig.ConfigValues[7] == '0')
    Serial.println(F("None sensor HYT"));
  else if (eepromConfig.ConfigValues[7] == '1')
  {
    SHT31sen = true;
    Serial.println(F("SHT31 sensor"));
  }
  else if (eepromConfig.ConfigValues[7] == '2')
  {
    AM2320sen = true;
    Serial.println(F("AM2320 sensor"));
  }

  Serial.print(F("eepromConfig.ConfigValues[8]: "));
  Serial.println(eepromConfig.ConfigValues[8]);
  SPS30sen = false;
  SEN5Xsen = false;
  PMSsen = false;
  AdjPMS = false;
  if (eepromConfig.ConfigValues[8] == '0')
    Serial.println(F("None PM sensor"));
  else if (eepromConfig.ConfigValues[8] == '1')
  {
    SPS30sen = true;
    Serial.println(F("SPS30 sensor"));
  }
  else if (eepromConfig.ConfigValues[8] == '2')
  {
    SEN5Xsen = true;
    Serial.println(F("SEN5X sensor"));
  }
  else if (eepromConfig.ConfigValues[8] == '3')
  {
    PMSsen = true;
    Serial.println(F("PMS sensor"));
  }
  else if (eepromConfig.ConfigValues[8] == '4')
  {
    AdjPMS = true;
    PMSsen = true;
    Serial.println(F("PMS sensor with stadistical adjust"));
  }

  // SPS30sen = 1
  // SEN5Xsen = 2
  // PMSsen = 4
  // AdjPMS = 8
  // SHT31sen = 16
  // AM2320sen =32
  // TDisplay = 256
  // OLED66 = 512
  // OLED96 = 1024
  // ExtAnt (External Antenna)= 2048
  // AmbInOutdoors (Indoors) = 4096
  // Brownout trick (true) = 8192

  if (SPS30sen)
    IDn = IDn + 1;
  if (SEN5Xsen)
    IDn = IDn + 2;
  if (PMSsen)
    IDn = IDn + 4;
  if (AdjPMS)
    IDn = IDn + 8;
  if (SHT31sen)
    IDn = IDn + 16;
  if (AM2320sen)
    IDn = IDn + 32;

  if (TDisplay)
    IDn = IDn + 256;
  if (OLED66)
    IDn = IDn + 512;
  if (OLED96)
    IDn = IDn + 1024;
  if (ExtAnt)
    IDn = IDn + 2048;
  if (AmbInOutdoors)
    IDn = IDn + 4096;
#if BrownoutOFF
  IDn = IDn + 8192;
#endif
#if ESP8266
  IDn = IDn + 16384;
#endif
  IDn = IDn + (Swver * 65536);
  Serial.print(F("IDn: "));
  Serial.println(IDn);
}

#endif

#if !SDyRTC

void Read_EEPROM()
{
#if ESP8266

  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyConfigStruct));
  // Check if the EEPROM contains valid data from another run
  // If so, overwrite the 'default' values set up in our struct
  if (EEPROM.percentUsed() >= 0)
  {
    Serial.println(F("EEPROM has data from a previous run."));
    Serial.print(EEPROM.percentUsed());
    Serial.println(F("% of ESP flash space currently used"));

    // Read saved data
    EEPROM.get(0, eepromConfig);
    Print_Config();
  }
  else
  {
    aireciudadano_device_id.toCharArray(eepromConfig.aireciudadano_device_name, sizeof(eepromConfig.aireciudadano_device_name)); // Initialize aireciudadano_device_name with aireciudadano_device_id
    Serial.println(F("No EEPROM data - using default config values"));
  }

#else
  // Read AireCiudadano device persistent info
  if (preferences.getBytesLength("config") > 0)
  {
    boolean result = preferences.getBytes("config", &eepromConfig, sizeof(eepromConfig));
    if (result)
    {
      Serial.println(F("Config data read from flash"));
    }
    else
    {
      Serial.println(F("Config data could not be read from flash"));
    }
  }
  else
  {
    aireciudadano_device_id.toCharArray(eepromConfig.aireciudadano_device_name, sizeof(eepromConfig.aireciudadano_device_name)); // Initialize aireciudadano_device_name with aireciudadano_device_id
    Serial.println(F("No EEPROM data - using default config values"));
  }

#endif
}

void Write_EEPROM()
{

#if ESP8266
  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyConfigStruct));

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromConfig);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "EEPROM Commit OK" : "EEPROM Commit failed");

#else
  // Write AireCiudadano device persistent info
  boolean result = preferences.putBytes("config", &eepromConfig, sizeof(eepromConfig));
  if (result)
  {
    Serial.println(F("Config data written to flash"));
  }
  else
  {
    Serial.println(F("Config data could not be written to flash"));
  }

#endif
}

void Wipe_EEPROM()
{ // Wipe AireCiudadano device persistent info to reset config data

#if ESP8266
  boolean result = EEPROM.wipe();
  if (result)
  {
    Serial.println(F("All EEPROM data wiped"));
  }
  else
  {
    Serial.println(F("EEPROM data could not be wiped from flash store"));
  }

#else

  boolean result = preferences.clear();
  if (result)
  {
    Serial.println(F("All EEPROM data wiped"));
  }
  else
  {
    Serial.println(F("EEPROM data could not be wiped from flash store"));
  }

#endif
}

#endif

#if Tdisplaydisp

#if Bluetooth
void displayBatteryLevel(int colour)
{ // Draw a battery showing the level of charge

  // Measure the battery voltage
  battery_voltage = ((float)analogRead(ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);

  Serial.print(F("battery voltage: "));
  Serial.println(F(battery_voltage));

  //  tft.drawString(String(battery_voltage), 42, 218);

  // If battery voltage is up 4.5 then external power supply is working and battery is charging
  if (battery_voltage > USB_Voltage)
  {
    tft.drawRect(3, 216, 34, 22, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
    tft.fillRect(21, 220, 5, 14, colour);
    tft.fillRect(28, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_1)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
    tft.fillRect(21, 220, 5, 14, colour);
    tft.fillRect(28, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_2)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
    tft.fillRect(21, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_3)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_4)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
  }
  else
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);

    // Measurements are not trustable with this battery level
    Serial.println(F("Battery level too low"));
  }
}
#endif

#endif

#if !ESP8266

void Suspend_Device()
{
  if (NoiseBUTTONFlag == false)
  {
    Serial.println(F("Presiona de nuevo el boton para despertar"));
    // Off sensors
    digitalWrite(OUT_EN, LOW); // step-up off

#if Tdisplaydisp

    if (TDisplay == true)
    {
      // #if !ESP8266
      //  int r = digitalRead(TFT_BL);
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Presiona boton", tft.width() / 2, tft.height() / 2 - 15);
      tft.drawString("para despertar", tft.width() / 2, tft.height() / 2 + 15);
      espDelay(3000);
      // digitalWrite(TFT_BL, !r);
      tft.writecommand(TFT_DISPOFF);
      tft.writecommand(TFT_SLPIN);
      // #endif
    }
#else
    espDelay(3000);
#endif

    // After using light sleep, you need to disable timer wake, because here use external IO port to wake up
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    // esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW);
  }
  else
    Serial.println(F("Tecla fallida, presione 1 segundo para despertar"));

  // After using light sleep, you need to disable timer wake, because here use external IO port to wake up
  //  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  // esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW);

  // set top button for wake up
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // Top button
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // Bottom button

  delay(200);
  esp_deep_sleep_start();
}

void print_reset_reason(RESET_REASON reason)
{
  switch (reason)
  {
  case 1:
    Serial.println(F("POWERON_RESET"));
    ResetFlag = true;
    DeepSleepFlag = false;
    break; /**<1,  Vbat power on reset*/
  case 3:
    Serial.println(F("SW_RESET"));
    break; /**<3,  Software reset digital core*/
  case 4:
    Serial.println(F("OWDT_RESET"));
    break; /**<4,  Legacy watch dog reset digital core*/
  case 5:
    Serial.println(F("DEEPSLEEP_RESET"));
    DeepSleepFlag = true;
    ResetFlag = false;
    break; /**<5,  Deep Sleep reset digital core*/
  case 6:
    Serial.println(F("SDIO_RESET"));
    break; /**<6,  Reset by SLC module, reset digital core*/
  case 7:
    Serial.println(F("TG0WDT_SYS_RESET"));
    break; /**<7,  Timer Group0 Watch dog reset digital core*/
  case 8:
    Serial.println(F("TG1WDT_SYS_RESET"));
    break; /**<8,  Timer Group1 Watch dog reset digital core*/
  case 9:
    Serial.println(F("RTCWDT_SYS_RESET"));
    break; /**<9,  RTC Watch dog Reset digital core*/
  case 10:
    Serial.println(F("INTRUSION_RESET"));
    break; /**<10, Instrusion tested to reset CPU*/
  case 11:
    Serial.println(F("TGWDT_CPU_RESET"));
    break; /**<11, Time Group reset CPU*/
  case 12:
    Serial.println(F("SW_CPU_RESET"));
    ResetFlag = false;
    DeepSleepFlag = false;
    break; /**<12, Software reset CPU*/
  case 13:
    Serial.println(F("RTCWDT_CPU_RESET"));
    break; /**<13, RTC Watch dog Reset CPU*/
  case 14:
    Serial.println(F("EXT_CPU_RESET"));
    break; /**<14, for APP CPU, reseted by PRO CPU*/
  case 15:
    Serial.println(F("RTCWDT_BROWN_OUT_RESET"));
    break; /**<15, Reset when the vdd voltage is not stable*/
  case 16:
    Serial.println(F("RTCWDT_RTC_RESET"));
    break; /**<16, RTC Watch dog reset digital core and rtc module*/
  default:
    Serial.println(F("NO_MEAN"));
  }
}

#endif

#if (OLED96display || OLED66display)

void displayInit()
{
  u8g2.setBusClock(100000);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setContrast(255);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.setFontMode(0);
  dw = u8g2.getDisplayWidth();
  dh = u8g2.getDisplayHeight();
  //  Serial.println(F("OLED display ready"));
}

void showWelcome()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 0, "AireCiudadano");
  u8g2.sendBuffer();
  u8g2.drawStr(0, 8, "ver: ");
  u8g2.sendBuffer();
  u8g2.drawStr(22, 8, sw_version.c_str());
  u8g2.sendBuffer();
  lastDrawedLine = 10;
  Serial.println(F("OLED display ready"));
  u8g2.sendBuffer();
}

void welcomeAddMessage(String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, lastDrawedLine, msg.c_str());
  lastDrawedLine = lastDrawedLine + 7;
  u8g2.sendBuffer();
}

void AddMessage(String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(7, lastDrawedLine, msg.c_str());
  u8g2.sendBuffer();
}

void displayCenterBig(String msg)
{
  if (dw > 64)
  {
    u8g2.setCursor(dw - 64, 6);
    u8g2.setFont(u8g2_font_inb24_mn);
  }
  else
  {
    u8g2.setCursor(dw - 28, 9);
    u8g2.setFont(u8g2_font_9x18B_tf);
  }
  u8g2.print(msg.c_str());

  u8g2.setCursor(100, 37);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.print("ug/m3");
}

void displayBottomLine(String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.print(msg);
}

void displayEmoticonLabel(int cursor, String msg)
{
  u8g2.setFont(u8g2_font_unifont_t_emoticons);
  u8g2.drawGlyph(76, 12, cursor);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(77, 17);
  u8g2.print(msg);
}

void displayTextLevel(String msg)
{
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(29, 31); //(35, 26);; (25, 29); (30, 29); (29, 28); (25, 30)(30, 29)
  u8g2.print(msg);        // 4 8 7 6 7 6
}

void displayColorLevel(int cursor, String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(35, 22);
  u8g2.print(msg);
}

#endif

#if !ESP8266
#if Tdisplaydisp

void displayAverage(int average)
{
  tft.setTextSize(1);

  if (average < 13)
  {

    tft.fillScreen(TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceGoodBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("GOOD", 34, 97);
  }
  else if (average < 36)
  {

    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    delay(50);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceModerateBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("MODERATE", 18, 97);
  }
  else if (average < 56)
  {

    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthySGroupsBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("UNHEALT SEN", 5, 97);
  }
  else if (average < 151)
  {

    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("UNHEALTHY", 13, 97);
  }
  else if (average < 251)
  {
    tft.fillScreen(TFT_VIOLET);
    tft.setTextColor(TFT_WHITE, TFT_VIOLET);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceVeryUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("VERY UNHEAL", 5, 97);
  }
  else
  {
    tft.fillScreen(TFT_BROWN);
    tft.setTextColor(TFT_WHITE, TFT_BROWN);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceHazardousBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("HAZARDOUS", 13, 97);
  }

  // Draw PM25 number
  tft.setTextSize(1);
  tft.setFreeFont(FF95);

  if (average < 10)
    tft.drawString(String(average), 45, 116);
  else if (average < 100)
    tft.drawString(String(average), 21, 116);
  else
    tft.drawString(String(average), 0, 116);

  // Draw PM25 units
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  tft.drawString("PM2.5: ", 30, 197);
  tft.drawString(String(round(PM25_value), 0), 90, 197);

  if (temp != 0 || humi != 0)
  {
    // Draw temperature
    tft.drawString("T" + String(temp), 60, 220);

    // Draw humidity
    tft.drawString("H" + String(humi), 95, 220);
  }
  else
    tft.drawString("ug/m3", 72, 218);
    // tft.drawString("ug/m3", 72, 268);

#if Wifi
  int rssi;
  rssi = WiFi.RSSI();

  if (rssi != 0)
  {
    if (pm25int < 57)
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_BLACK);
    else
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_WHITE);
    Serial.print(F(" RSSI: "));
    Serial.print(F(rssi));
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(F(rssi));
    tft.drawString(String(rssi), 30, 220);
  }
#endif
}
#endif
#endif

#if (OLED96display || OLED66display)

void displaySensorAverage(int average)
{
  if (average < 13)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceGood);
    displayColorLevel(0, " green");
    displayTextLevel("  GOOD");
  }
  else if (average < 36)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceModerate);
    displayColorLevel(0, "yellow");
    displayTextLevel("MODERATE");
  }
  else if (average < 56)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceUnhealthySGroups);
    displayColorLevel(0, "orange");
    displayTextLevel("UNH SEN");
  }
  else if (average < 151)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceUnhealthy);
    displayColorLevel(0, "  red");
    displayTextLevel("UNHEALT");
  }
  else if (average < 251)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceVeryUnhealthy);
    displayColorLevel(0, "violet");
    displayTextLevel("V UNHEA");
  }
  else
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceHazardous);
    displayColorLevel(0, " brown");
    displayTextLevel(" HAZARD");
  }
  char output[4];
  sprintf(output, "%03d", average);
  displayCenterBig(output);
}

// TODO: separate this function, format/display
void displaySensorData(int pm25, int humi, int temp, int rssi)
{
  char output[22];
  sprintf(output, "%03d H%02d T%02d", pm25, humi, temp); // 000 E00 H00% T00°C
  u8g2.setCursor(dw / 2 + 16, 48);
  displayBottomLine(String(output));

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(43, 1);
  sprintf(output, "%04d", pm25); // PM25 instantaneo fuente pequeña
  u8g2.print(output);

#if Wifi
  u8g2.setCursor(20, dh - 6);

  if (rssi == 0)
  {
    u8g2.print("   ");
    Serial.println(F(""));
  }
  else
  {
    u8g2.drawBitmap(5, dh - 8, 1, 8, Icono_wifi_on);
    Serial.print(F(" RSSI: "));
    Serial.print(F(rssi));
    //    rssi = abs(rssi);
    sprintf(output, "%02d", rssi);
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(F(rssi));
    u8g2.print(rssi);
  }
#endif
}

void pageStart()
{
  u8g2.firstPage();
}

void pageEnd()
{
  u8g2.nextPage();
}

#endif

#if Bluetooth
void Write_Bluetooth()
{ // Write measurements to Bluetooth

  uint32_t ValSampleIntervals;

  gadgetBle.writePM2p5(pm25int);
  gadgetBle.writeTemperature(temp);
  gadgetBle.writeHumidity(humi);
  Serial.println(F("Bluetooth frame: PM25, humidity and temperature"));
  gadgetBle.commit();

  ValSampleIntervals = gadgetBle.getSampleInterval();
  //  Serial.print(F("ValSampleIntervals: "));
  //  Serial.println(F(ValSampleIntervals));

  //  Serial.print(F("Bluetooth_loop_time: "));
  //  Serial.println(F(Bluetooth_loop_time));

  Bluetooth_loop_time = ValSampleIntervals;

  if (eepromConfig.BluetoothTime != Bluetooth_loop_time)
    FlashBluetoothTime();
}
#endif

#if (SDyRTC || SaveSDyRTC)
void Write_SD()
{ // Write date - time and measurements to SD card

  DateTime now = rtc.now();

  char buf1[] = "YYYY/MM/DD_hh:mm:ss";
  String dataString = "";

  // make a string for assembling the data to log:

  dataString = now.toString(buf1);
  dataString += "_PM:";
  dataString += pm25int;
  if (SHT31sen == true || AM2320sen == true)
  {
    dataString += "_H:";
    dataString += humi;
    dataString += "_T:";
    dataString += temp;
  }

  dataFile.println(dataString);

  // print to the serial port too:
  Serial.print(F("Data SD: "));
  Serial.println(F(dataString));

  //  Serial.print(F("Unixtime = "));
  //  Serial.print(F(now.unixtime()));
  //  Serial.println(F(" s"));
  //  Serial.print(F(now.unixtime() / 86400L));
  //  Serial.println(F("d"));

  dataFile.flush();
}
#endif