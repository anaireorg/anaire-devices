# anaire-devices

SW for anaire devices: Get CO2, temperature and humidity measurements and send them to the anaire cloud app

From 20201109 ANAIRE DEVICE CODE - anaire@anaire.org

Files:
- anaire-device.nodemcuv2.ino: code for prototype on AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2
- anaire_config.h: device configuration info
  device id, initial thresholds, WiFi setup, cloud app endpoint
- mh-z14a_co2-manual-v1_01.pdf: CO2 sensor datasheet
- NodeMCU_LUA_Amica_V2_Modul_mit_ESP8266_12E_Datenblatt.pdf: nodemcu datasheet
- NodeMCU_LUA_Amica_V2_Schematic.pdf: nodemcu schematic
- Anaire Prototipo MH-Z14a.fzz: fritzing cabling design

Parts:
- AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
- MH-Z14A - CO2 sensor. Connected by serial port (swSerial on NodeMCU) http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf 
- AZ-Delivery DHT11 Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
- AZ-Delivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
- AZ-Delivery 0.91 inch OLED I2C Display 128 x 32 Pixels  https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display

Parts list for a complete prototype in amazon: https://www.amazon.es/hz/wishlist/ls/8NAKLGML187W?ref_=wl_share

Arduino IDE Setup:
 - Start Arduino and open Preferences window.
 - Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
 - Open Boards Manager from Tools > Board menu and find esp8266 platform by esp8266 community and install the software for Arduino from a drop-down box.
 - Select "NodeMCU 1.0" board from Tools > Board menu after installation

Install the following libraries in Arduino IDE:
- ESP8266WiFi for WiFi https://github.com/bportaluri/WiFiEsp
- DHTesp - to manage DHT11 or DHT22 temperature and humidity sensors https://github.com/markruys/arduino-DHT
- EspSoftwareSerial - to manage sw serial port to communicate with CO2 sensor https://github.com/plerup/espsoftwareserial/
- ArduinoMqttClient - for MQTT communications https://github.com/arduino-libraries/ArduinoMqttClient
- esp8266-oled-ssd1306 for oled display https://github.com/ThingPulse/esp8266-oled-ssd1306

Design:
- Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector) is also connected to the external buzzer
  * When CO2 Status is "ok" (below warning threshold) LED keeps ON and buzzer is off
  * When CO2 Status is "warning" builtin LED and external buzzer alternate at a slow pace (WARNING_BLINK_PERIOD)
  * When CO2 Status is "alarm" builtin LED and external buzzer alternate at fast pace (ALARM_BLINK_PERIOD)
  This GPIO16 is also used when downloading SW to the nodemcu board: the external buzzer sounds while downloading to the board
- Flash button, to the risht of the USB connector, allows to toggle between activating/deactivating local alarms (visual and sound)
  * It also shows device ID and IP address after being pressed until a new CO2 measurement is displayed
- CO2 sensor is connected through a software serial port using GPIO13(D7) and GPIO15 (D8)
- DHT11 humidity and temperature sensor is connected on GPIO 5 (D1)
- SSD1306 OLED display is connected on GPIO14(D5) and GPIO12 (D6)
- The device is designed to work only with the CO2 sensor, so buzzer, DHT11 humidity and temperature sensor, and OLED display are optional
- The device is designed to recover from Wifi, MQTT or sensor reading temporal failures

