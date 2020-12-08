# anaire-devices

SW for anaire devices: Get CO2, temperature and humidity measurements and send them to the anaire cloud app

From 20201109 ANAIRE DEVICE CODE - anaire@anaire.org

anaireorg/anaire-devices is licensed under the GNU General Public License v3.0

![Dispositivo](https://github.com/anaireorg/anaire-devices/blob/main/Dispositvo_ANAIRE_display.png)

# Files
- anaire-device.nodemcuv2.ino: code for prototype on AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2
- anaire_config.h: device configuration info
  device id, initial thresholds, WiFi setup, cloud app endpoint
- mh-z14a_co2-manual-v1_01.pdf: CO2 sensor datasheet
- NodeMCU_LUA_Amica_V2_Modul_mit_ESP8266_12E_Datenblatt.pdf: nodemcu datasheet
- NodeMCU_LUA_Amica_V2_Schematic.pdf: nodemcu schematic
- Anaire Prototipo MH-Z14a.fzz: fritzing cabling design

# Parts
- AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
- MH-Z14A - CO2 sensor. Connected by serial port (swSerial on NodeMCU) http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf 
- AZ-Delivery DHT11 Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
- AZ-Delivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
- AZ-Delivery 0.91 inch OLED I2C Display 128 x 32 Pixels  https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display

Parts list for a complete prototype in amazon: https://www.amazon.es/hz/wishlist/ls/8NAKLGML187W?ref_=wl_share

# Arduino IDE Setup
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

# Design and usage
- Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector) is also connected to the external buzzer
  - When CO2 Status is "ok" (below warning threshold) LED keeps ON and buzzer is off
  - When CO2 Status is "warning" builtin LED and external buzzer alternate at a slow pace (WARNING_BLINK_PERIOD)
  - When CO2 Status is "alarm" builtin LED and external buzzer alternate at fast pace (ALARM_BLINK_PERIOD)
  This GPIO16 is also used when downloading SW to the nodemcu board: the external buzzer sounds while downloading to the board
- Flash button, to the risht of the USB connector, allows to toggle between activating/deactivating local alarms (visual and sound)
  - It also shows device ID and IP address after being pressed until a new CO2 measurement is displayed
- CO2 sensor is connected through a software serial port using GPIO13(D7) and GPIO15 (D8)
- DHT11 humidity and temperature sensor is connected on GPIO 5 (D1)
- SSD1306 OLED display is connected on GPIO14(D5) and GPIO12 (D6)
- The device is designed to work only with the CO2 sensor, so buzzer, DHT11 humidity and temperature sensor, and OLED display are optional
- The device is designed to recover from Wifi, MQTT or sensor reading temporal failures
- The web server is activated, therefore entering the IP on a browser allows to see the device data. The IP is showed during boot and after pressing the button.
  - The hostname in the form <anaire_device_id>.local is definied by mDNS. But it will depend on the local router to be operative or not.
- The web server allows to calibrate the CO2 sensor. Put the device on a clean environment (open air) and click the option in the web server. Waits during the calibration countdown time (20 minutes)

# Montaje

1. Soldar los cuatro pines en el sensor de CO2 donde está indicado en la imagen, para poder pincharlo en la breadboard

![Pines del sensor de CO2 MHZ-14A](https://github.com/anaireorg/anaire-devices/blob/main/Pines_CO2_ANAIRE.png)

2. Disponer y cablear los elementos como se muestra en las siguientes imágenes. Se recomienda poner unas gotitas de pegamento en la base de la fuente de alimentación, para que no se pueda mover de su sitio:

Componentes:
![Montaje](https://github.com/anaireorg/anaire-devices/blob/main/medida_photo_2020-12-07_22-28-34.png)

Conexiones:
![Conexiones](https://github.com/anaireorg/anaire-devices/blob/main/Connections_NodeMCU_LUA_Amica_V2_Schematic.png)

3. Cargar los archivos anaire-device.nodemcuv2.ino y anaire_config.h en el IDE de Arduino.

4. Editar el archivo anaire_config.h para configurar:
  - el id del dispositivo 
  - los umbrales de aviso y alarma de nivel de CO2 (por defecto 700 y 1000 respectivamente)
  - el ssid y la contraseña de la red Wifi
  - el endpoint de la app en la nube <nombrededominio>:30183 (la app se ha debido desplegar previamente)
  
5. Conectar el puerto USB y descargar los archivos a la tarjeta NodeMCU. El zumbador emitirá sonido durante la descarga del SW (varios segundos)

6. Esperar a que se complete la inicialización del dispositivo (cuando termine la cuenta atrás en el display) y que aparezcan las primeras medidas. Comprobar que los datos del dispositivo están accesibles en la app en el enlace <nombrededominio>:3300

7. Pulsar el botón de "Flash" en la NodeMCU para comprobar el ID y obtener la dirección IP asignada

8. Calibración del dispositivo:
  - Colocar el dispositivo en un entorno limpio (en el exterior o al lado de una ventana abierta, con ventilación abundante)
  - En un navegador conectado en la misma red WiFi que el dispositivo introducir la dirección IP. Se obtendrá una imagen como la siguiente:
  
  ![Web server](https://github.com/anaireorg/anaire-devices/blob/main/Captura_calibration.PNG)
  
  - Hacer click en "here" en el enlace mostrado en la primera línea, y esperar a que se termine la cuenta atrás en el display del dispositivo y se vuelvan a mostrar medidas. El proceso se completa en aproximadamente 20 minutos.

![Calibrating](https://github.com/anaireorg/anaire-devices/blob/main/calibrating_photo_2020-12-07_22-01-00.jpg)

**Una vez calibrado el dispositivo está listo para su uso. Aún en caso de errores de conexión WiFi o MQTT el dispositivo seguirá mostrando las medidas actualizadas de CO2 en el display. El proceso de calibración debe repetirse una vez cada 30 días.**

# Errores mostrados en el display
  - err_wifi: no se pudo conectar a la red WiFi. Compruebe los detalles en el archivo anaire_config.h.
  - err_mqtt: no se pudo conectar al endpoint de la app en la nube. Compruebe los detalles en el archivo anaire_config.h.
  - err_co2: no se pudo conectar con el sensor de CO2. Compruebe las conexiones.
  - err_dht: no se pudo conectar con el sensor de humedad y temperatura. Compruebe las conexiones.

