# Medidor ANAIRE: CO2 (NDIR), temperatura y humedad
*anaireorg/anaire-devices is licensed under the GNU General Public License v3.0*

*@Asociación Anaire www.anaire.org anaire@anaire.org* 

Proyecto libre y abierto (opensource), tanto hardware como software, para la fabricación de un medidor de CO2, temperatura y humedad, de mínimo coste y alta precisión, conectado por WiFi a una aplicación en la nube denominada Anaire Cloud App para mostrar los valores instantáneos y el histórico de mediciones de CO2, temperatura y humedad, facilitando así el análisis de la infortmación de las medidas y su correlación con protocolos de ventilación anti COVID-19.

El concepto se muestra en la siguiente imagen. Se trata de un dispositivo basado en un microcontrolador ESP8266, al que se conectan sensores para la medida de CO2, temperatura y humedad. El dispositivo tiene un display para mostrar las medidas e indicaciones de estado, un LED y un zumbador para poder emitir alertas visuales y sonoras, y un botón para poder deshabilitar la alerta local. Dispone de conectividad WiFi para poder enviar las mediciones realizadas a una aplicación en la nube, Anaire Cloud (https://github.com/anaireorg/anaire-cloud), que permite agregar la información de múltiples dispositivos y acceder al histórico de medidas desde cualquier dispositivo vía Internet.
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/Dispositivo_ANAIRE.png" width="80%" height="80%"/>
</p>

En este repositorio se publica información completa para la fabricación del medidor de CO2 Anaire, desde el código fuente hasta las especificaciones de componentes, cableados y opciones de montaje tanto en placa de prototipado como utilizando cajas de impresión 3D para la disposición de los componentes. La intención es que cualquier persona pueda encontrar aquí la información necesaria y suficiente para montar su propio sensor de CO2, con o sin conexión a la nube, sin necesidad de grandes conocimientos técnicos, como ayuda para combatir la pandemia causada por la COVID-19, ayudando a preparar entornos más seguros, con mínimos niveles de CO2, que puedan ayudar a prevenir contagios en entornos cerrados.
  
Toda la información se publica en formato "open source", tanto diseño hardware como código software, con el ánimo de facilitar el uso y difusión de la tecnología todo lo posible. Con el mismo espíritu se tratan de resolver todas las incidencias reportadas en los repositorios de Anaire (*issues*).

---  

# Versión actual
21.01 ladinadaina (enero de 2021)

# Características principales
* Medición de CO2 mediante sensores NDIR de alta precisión
  * Adicionalmente se realizan de forma simultánea mediciones de temperatura y de humedad
    
* Completamente opensource, tanto hardware como software. Toda la información, tanto sobre este dispositivo como sobre la aplicación en la nube con la que se comunica, está disponible en los repositorios de Anaire en github (https://github.com/anaireorg). Sin excepciones.  
  
* Alarma local, visual y sonora, cuando el nivel de CO2 sobrepara los umbrales de aviso (700 ppm por defecto) y alarma (1000 ppm por defecto)
  * Esta alarma puede conmutarse localmente (encendido/apagado) mediante un botón en el dispositivo o desde la configuración del dispositivo en la nube.  
    
* Conexión a una red WiFi para el envío de las medidas a una aplicación desplegada en la nube con objeto de obtener los siguientes beneficios adicionales:
  * Agregar la información de múltiples dispositivos permitiendo visualizar de forma simultánea el estado de todos ellos, siguiendo un código de colores sencillo (verde, amarillo y rojo) para indicar el estado actual de una organización con múltiples dispositivos. La idea es, sobre todo, poder monitorizar la medida del CO2 en centros de enseñanza, donde alumnos y profesores deben compartir espacios cerrados
  * Almacenar las medidas de CO2, temperatura y humedad realizadas cada 30 segundos durante al menos 15 días.
  * Facilitar el análisis de las medidas almacenadas, permitiendo el acceso al histórico de medidas de forma sencilla, para de este modo facilitar el análisis del funcionamiento de los protocolos de ventilación.  
   
* Toda la información almacenada en la nube es accesible vía Internet, tanto en modo individual para cada dispositivo como en modo agregado en cuadros de mandos configurables con usuarios y permisos de visualización y de edición arbitrarios. De este modo se pueden definir distintos tipos de usuarios con distintos niveles de visualización de la información: personal del centro, alumnos, padres de alumnos, etc.  
  
* Para el acceso a la información sólo es preciso un dispositivo con conexión a Internet (ordenador, teléfono móvil, tableta, etc.), la url de acceso al dispositivo o a la organización, y un usuario y contraseña (en caso de que se hayan habilitado políticas de acceso).  
  
* Comunicación entre el dispositivo y la aplicación en la nube mediante protocolo MQTT securizado con TLS.  
  
* Fabricación sencilla, simplemente "pinchando" los componentes en placas de prototipado ("breadboard"), soldando únicamente los cuatro pines necesarios en el sensor de CO2. En el caso del sensor SCD30, es especialmente sencillo, ya que tras soldar los cuatro pines necesarios se puede "pinchar" el sensor en la placa de prototipado de forma alineada y consecutiva con la tarjeta de control NodeMCU, ahorrando así esos cables y proporcionando mayor robustez al montaje.
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireSlim.jpeg" width="30%" height="30%" />
</p>  
  
* Alternativa para la fabricación mucho más robusta utilizando cajas diseñadas a medida y fabricadas mediante impresoras 3D, y simplemente conectando los componentes mediante cables dupont hembra y encajándolos en los espacios preparados para ello en la caja. La caja es compatible para los dos sensores, y alberga adicionalmente el display OLED (y el resto de los componentes) de forma que sea muy fácil la lectura de las medidas. Todos los detalles para la fabricación de la caja se pueden encontrar en la carpeta "3D print" en este repositorio.  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxFrontal.jpg" width="30%" height="30%" />
</p>  
  
* Alimentación a través del puerto Micro USB de la tarjeta de control NodeMCU LUA Amica V2.   
  
* Actualización remota de parámetros (umbrales de aviso y de alarma, aviso local de alarma, etc.) desde la aplicación en la nube.    
  
* Portal cautivo para la configuración de la red WiFi en la localización final, sin necesidad de modificar el software del dispositivo. Accesible mediante botón en el dispositivo.  
  
* Actualización remota del SW del dispositivo, iniciada desde la aplicación en la nube. Permite la actualización automática a la última versión del software en modo binario, almacenada en este repositorio en github: https://github.com/anaireorg/anaire-devices/blob/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin    
  
# Hardware
El medidor Anaire es un dispositivo basado en un microcontrolador ESP8266 dispuesto en una tarjeta de control AZ Delivery NodeMCU Lua Amica V2, que proporciona también conectividad WiFi, y permite su programación desde el IDE de Arduino. Para realizar las medidas se conectan sensores de CO2, temperatura y humedad. Adicionalmente se conectan un display para mostrar las mediciones e indicaciones de estado, y un zumbador para poder emitir alertas sonoras. 
  
Con objeto de simplificar la fabricación y no añadir más componentes aun proporcionando máxima funcionalidad, se utilizan los siguientes elementos ya disponibles en la tarjeta NodeMCU:
  
* Se emplea uno de los dos LED incorporados a la tarjeta NodeMCU para proporcionar alertas visuales. El LED está apagado normalmente; parpadea lentamente cuando el dispositivo está en estado de aviso por CO2, y parpadea rápidamente en caso de encontrarse en alarma. Las frecuencias de parpadeo son iguales a las de la alerta sonora proporcionada por el zumbador.

* Se emplea el botón de Flash (a la derecha del conector Micro USB) poder deshabilitar la alerta local. Y para volverla a habilitar, ya que el botón conmuta entre ambos estados.
  * Adicionalmente, cada vez que se presiona el botón de Flash se muestran el modelo, ID y dirección IP del dispositivo, hasta la realización de la siguiente medición.

* Presionando dos veces consecutivas el botón de Reset (a la izquierda del conector MicroUSB) el dispositivo se reinicia en modo de portal cautivo, lo que permite la configuración de la red WiFi y el acceso a otros parámetros de configuración.
  
El dispositivo es plenamente operativo incluso sin el display y sin el zumbador. Para funcionar con mínimo coste sólo es necesaria una tarjeta de control NodeMCU y un sensor de CO2, todos los demás elementos son opcionales.

Para los sensores de CO2 existen dos alternativas. Actualmente se soportan dos sensores de CO2, ambos con tecnología NDIR: el Sensirion SCD30 (https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-co2/) y el Winsen MH-Z14A (http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf). 
  
El Sensirion SCD30 es un sensor de mayor calidad: tiene mayor precisión (30 ppm frente a los 50 ppm del MH-Z14A); incorpora sensor de temperatura y humedad, con lo que no hay que añadir un sensor adicional para medir temperatura y humedad, facilitando la fabricación (además de permitir mecanismos de compensación, etc.); tiene un mecanismo de autocalibración más robusto, en el que se hace uso de las mediciones de los últimos 30 días, en lugar de las últimas 24 horas, como en el caso del MH-Z14A; su respuesta es también más rápida ante cambios atmosféricos (por ejemplo, al introducir ventilación en un espacio cerrado). El inconveniente del Sensirion SCD30 es que suele ser más caro y tiene menos opciones de compra y normalmente mayores plazos de entrega. El MH-Z14A está disponible en todo tipo de plataformas de comercio electrónico. Para comprar el Sensirion SCD30 hay que recurrir normalmente a plataformas más especializadas en componentes electrónicos.
  
El software del medidor de CO2 de Anaire es compatible con ambos sensores, y detecta automáticamente cuál de los dos está en uso, adaptándose a ello sin necesidad de realizar ningún cambio de configuración.
  
En caso de utilizar el MH-Z14A como medidor de CO2 hemos incorporado el AZ Delivery DHT11 como sensor de temperatura y humedad. En caso de utilizar el SCD30 no es necesario incorporar ningún componente adicional.
  
**Nuestra recomendación es utilizar el Sensirion SCD30 siempre que sea posible, ya que permite fabricar, por un coste muy similar, un dispositivo de mayor precisión y estabilidad en el tiempo**. En cualquier caso se recomienda encarecidamente el análisis de la documentación técnica de ambos sensores, especialmente para la interpretación de las medidas y la determinación de procedimientos de recalibración, en caso de considerarlo necesario para optimizar la evolución de la precisión de las medidas con el paso del tiempo.
  
La alimentación del dispositivo se realiza directamente a través del puerto Micro USB de la tarjeta de control NodeMCU LUA Amica V2, el mismo que se utiliza para programarla y para observar mensajes de diagnóstico. Se recomienda utilizar fuentes de alimentación (enchufes USB, puertos USB en ordenadores, etc.) que puedan proporcionar al menos 500 mA (que es lo más frecuente, aunque podría no ser así en el caso de antiguos cargadores de teléfonos móviles, por ejemplo).   

A continuación se enumeran los elementos citados, incluyendo enlaces a su documentación y a opciones para su adquisición.

## Elementos comunes
* Tarjeta de control basada en microcontrolador ESP8266: AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2  
https://www.az-delivery.de/es/products/nodemcu  
https://www.amazon.es/dp/B06Y1LZLLY/ref=twister_B082DJVXFC?_encoding=UTF8&psc=1  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/NodeMCU/NodeMCU%20transparente.png" width="20%" height="20%" />  
</p>
  
* Display OLED: AZDelivery 0.91 inch OLED I2C Display 128 x 32 Pixels  
https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display  
https://www.amazon.es/dp/B079H2C7WH/ref=twister_B082MC4QJ4?_encoding=UTF8&psc=1  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/Display/Display%20transparente.png" width="25%" height="25%" />  
</p>
  
* Zumbador: AZDelivery Active Buzzer  
https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r  
https://www.amazon.es/dp/B089QJKJXW/ref=twister_B082MHYNND?_encoding=UTF8&psc=1  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/Zumbador/Zumbador%20transparente.png" width="10%" height="10%" />  
</p>
  
## Sensores
* Anaire Slim: sensor de CO2, temperatura y humedad Sensirion SCD30 
https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-co2/  
https://www.digikey.es/product-detail/en/sensirion-ag/SCD30/1649-1098-ND/8445334  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/SCD30/SCD30%20transparente.png" width="20%" height="20%" />
</p>  
  
* Anaire Bread: sensor de CO2 Winsen MHZ14A
http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf  
https://www.amazon.es/MH-Z14A-di%C3%B3xido-infrarrojo-anal%C3%B3gica-ambiente/dp/B07CXGL7XG  
  
  * Adicionalmente, el AnaireBread necesita el sensor de temperatura y humedad AZ-Delivery DHT11
  https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
  https://www.amazon.es/dp/B089W8DB5P/ref=twister_B089YSBB1N?_encoding=UTF8&psc=1
  <p align="center">
    <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/MH-Z14A/MH-Z14A%20transparente.png" width="30%" height="30%" />
  </p>  

## Otros:
 * O bien caja AnaireBox imprimida por 3D, acompañada por cables dupont hembra-hembra  
 https://www.thingiverse.com/thing:4694633  
 https://www.amazon.es/SODIAL-Puente-Hembra-Soldadura-Flexible/dp/B00HUH9GOC/ref=sr_1_4?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=cable+dupont+hembra+hembra&qid=1609613291&s=industrial&sr=1-4  
 
 * O bien placas de prototipado
   * Placa de 400 puntos, para el AnaireSlim  
   https://www.amazon.es/dp/B071ZGC75Y/ref=twister_B07T88TTXF?_encoding=UTF8&psc=1  
     
   * Placa de 830 puntos, para el AnaireBread  
   https://www.amazon.es/dp/B071ZGC75Y/ref=twister_B07T88TTXF?_encoding=UTF8&psc=1  
     
   * En ambos casos son necesarios cables dupont macho-macho  
   https://www.amazon.es/Neuftech-jumper-20cm-Arduino-Breadboard/dp/B00NBNIETC/ref=sr_1_4?dchild=1&keywords=dupont+macho+macho&qid=1609613744&sr=8-4  
   
   
 * Será necesario un cable válido para conectar desde un puerto USB del ordenador al puerto Micro USB de la tarjeta NodeMCU, para poder descargar el software del dispositivo. Este mismo cable puede ser utilizado para la alimentación eléctrica desde un ordenador, o desde un enchufe USB o includo desde una batería portátil USB. También podría utilizarse, una vez descargado el software, un transformador de un antiguo teléfono móvil u otro tipo de dispositivo que tenga un conector Micro USB y proporcione al menos 500 mili amperios
https://www.amazon.es/TM-Electron-CXU201020-Cable-Blanco/dp/B07BQD6P74/ref=sr_1_22?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=usb+micro+usb&qid=1609613811&sr=8-22

# Software
El dispositivo se programa exactamente igual que una tarjeta arduino, utilizando el IDE (entorno de desarrollo) de Arduino. Hay que instalar de forma adicional algunos componentes de software, tanto en el PC utilizado como en el propio entorno de Arduino. A continuación se describe en detalle el procedimiento para poder preparar un entorno de desarrollo de SW operativo que permita la descarga del software en los dispositivos (y su programación para modificarlo, en caso deseado).

## Preparación del entorno de Arduino
  
 1. Instalar en el ordenador el driver USB-UART (conversión USB a puerto serie) para poder comunicarse con la tarjeta NodeMCU:
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
  
 2. Arrancar el IDE de Arduino, una vez descargado desde el siguiente enlace e instalado en el PC:
https://www.arduino.cc/en/software
  
 3. Abrir la ventana de preferencias (Archivo -> Preferencias)

 4. Para gestionar tarjetas como la NodeMCU, basadas en el microcontrolador ESP8266, introducir en el campo *Gestor de URLs adicionales de tarjetas* lo siguiente: http://arduino.esp8266.com/stable/package_esp8266com_index.json. Se pueden introducir URLs múltiples, separados por comas.

 5. Abrir *Herramientas -> Placa -> Gestor de tarjetas*, localizar *esp8266 platform by esp8266 community* e instalar el software desde la cxaja de selección

 6. Tras la instalación seleccionar en *Herramientas -> Placa* la opción *NodeMCU 1.0*, para de esta forma trabajar con esa tarjetas

 7. Instalar las siguientes Librerías, utilizando la opción *Herramientas -> Administrar Bibliotecas* del IDE de Arduino. Cuidado: se ha observado que puede fallar si se instalan directamente los archivos zip de las librerías y se descomprimen en el PC local. **Se recomienda encarecidamente instalar las librerías desde el IDE de Arduino**.

  - WiFiEsp https://github.com/bportaluri/WiFiEsp (uso de WiFi con ESP8266)
  - Arduino Client for MQTT https://pubsubclient.knolleary.net/ (cliente MQTT para comunicación con la nube)
  - ArduinoJson https://arduinojson.org/?utm_source=meta&utm_medium=library.properties (para procesar mensajes MQTT)
  - esp8266-oled-ssd1306 https://github.com/ThingPulse/esp8266-oled-ssd1306 (gestión del display OLED)
  - SparkFun_SCD30_Arduino_Library https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library (gestión del sensor de CO2, temperatura y humedad Sensirion SCD30)
  - DHTesp https://github.com/beegee-tokyo/DHTesp (gestión del sensor de temperatura y humedad DHT11)
  - EspSoftwareSerial https://github.com/plerup/espsoftwareserial/ (comunicación serie con el sensor de CO2 MH-Z14A)
  - ESP_EEPROM https://github.com/jwrw/ESP_EEPROM (almacenamiento de parámetros persistentes en la memoria flash de la NodeMCU)
  - WifiManager kentaylor https://github.com/kentaylor/WiFiManager (gestión de la conexión WiFi)
  - Double Reset detector https://github.com/datacute/DoubleResetDetector (detector de doble pulsación del botón reset para reinicio en el portal cautivo y poder configurar la WiFi y otros parámetros)

## Obtención del software del Medidor de CO2 de ANAIRE
El software está disponible en su última versión en el siguiente enlace:

Una vez configurado el entorno de desarrollo en el apartado anterior, cargar este programa en el entorno de Arduino, y descargarlo en la tarjeta haciendo click en el botón del IDE que muestra una flecha horizontal apuntando hacia la derecha

# Fabricación
Véase el archivo Montaje.md (enlace) para conocer las principales opciones de fabricación que se proponen, siempre tratando de simplificar al máximo manteniendo las prestaciones técnicas. En cualquier caso se proporciona toda la información necesaria para que cualquiera pueda plantearse alternativas de fabricación (por ejemplo, mediante placas PCB).
Una vez completado el montaje de los componentes, hay que proceder a la descarga del software en la tarjeta de control NodeMCU. Para ello realice los siguientes pasos:
* Conecte mediante un cable USB el dispositivo al ordenador en el que haya configurado el IDE de Arduino
* Compruebe que la tarjeta se ha detectado y se ha seleccionado el puerto apropiado
**Verifique que las variables de conexión a la aplicación de la nube son correctas:
**Presione el botón de descarga, como se muestra en la siguiente imagen
**Espere varios minutos. Primero compilación, luego descarga. Durante la descarga suena el pitido del zumbador (sirve como comprobación)
**Compruebe en el display que el dispositivo se ha reiniciado al término del pitido del zumbador. debería ver...,


# Configuración
* ID
* Configuración de la conexión WiFi
 * Tras pulsar 2 veces consecutivas el botón de reset, el dispositivo se reinicia en un portal cautivo con su propia red wifi***
 *Conectar a la red ESP_IdDispositivo y acceder con un navegador a la dirección 192.168.4.1 para seleccionar la red wifi e introducir la contraseña****


* Configuración de la conexión a la aplicación en la nube
** Enlaces de dispositivo y de organización
** Se podrá modificar la conexión a la nube en el portal cautivo WioFi en una versión posterior.

# Consideraciones de diseño
- Alarma local, display y zumbador
Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector) is also connected to the external buzzer
- Utilización de LEDs y botones en placa
- ID del dispositivo e integración con la aplicación de Anaire en la nube
- Dirección IP del dispositivo y portal Web. Hostname. - The hostname in the form <anaire_device_id>.local   is definied by mDNS. But it will depend on the local router to be operative or not.
- The web server allows to calibrate the CO2 sensor.
- Calibración. Put the device on a clean environment (open air) and click the option in the web server. Waits during the calibration countdown time (20 minutes)

# Instrucciones de uso
- Alimentar el dispositivo con un cable conectado al conector micro usb   
- El display mostrará el texto "anaire.org" durante unos segundos, y a continuación mostrará el nombre y el ID del dispositivo, junto a una cuenta atrás debida a que los sensores de CO2 requieren de un tiempo de estabilización y calentamiento para su puesta en marcha correcta   
- Una vez concluida la cuenta atrás, el dispositivo mostrará en el display la última medida de CO2, temperatura y humedad realizada, y un texto indicando si el CO2 está bien, regular o mal
 - En caso de que el valor medido de CO2 supero el umbral de aviso, el dispositivo empezará a emitir un pitido intermitente, así como el parpadeo del LED de estado de CO2, de forma alternativas
   - Los valores por defecto de los umbrales son 700 PPM para el aviso y 1.000 PPM para la alarma. Estos valores se pueden modificar desde la aplicación de Anaire en la nube

 - En caso de que el valor medido de CO2 supero el umbral de alarma, el dispositivo empezará a emitir un pitido intermitente, así como el parpadeo del LED de estado de CO2, de forma alternativas y con una frecuencia mayor que en el caso de aviso
 - Si se desea detener la indicación local de alarma (visual y sonora), presione una vez el botón de Flash (a la derecha del conector USB)
   - El display mostrará, de forma adicional, el modelo, el ID y la dirección IP del dispositivo
   - Si se vuelve a presional el botón de Flash, se reactivará el aviso de alarma local. Es decir, el botón de Flñash permite conmutar entre avisar o no de forma local cuando los valores de CO2 superen los umbrales estabilizaciónecidos
   - Cuando la medida de CO2 sea inferior al umbral de aviso, el dispositivo reseteará el estado de la señal local de alarma, de forma que automáticamente volverá a dar indicaciones locales de alarma si se vuelven a superar los umbrales de aviso o de alarma en el futuro, sin necesidad de reactivar la alarma local

 - Si hay algún error, la última línea del display mostrará el error, en lugar de dar una indicación del estado de CO2. El dispositivo está diseñado para recuperarse automáticamente de los errores cuando la causa que los provoca se ha resuelto. El usuario no tiene que hacer nada para recuperarse de los errores
 - Servidor Web
 - Portal cautivo
 - Calibración de los Sensores

## Errores mostrados en el display
  - err_wifi: no se pudo conectar a la red WiFi. Compruebe el estado presionando dos veces consecutivas el botón de Reset y conectando al portal cautivo, como se explica en el apartado de Configuración del dispositivo
  - err_mqtt: no se pudo conectar al endpoint de la app en la nube. Compruebe los detalles al inicio del código del dispositivo y verifique la conectividad del endpoint dwfinido
  - err_co2: no se pudo conectar con el sensor de CO2. Compruebe las conexiones
  - err_dht: no se pudo conectar con el sensor de humedad y temperatura DHT11. Compruebe las conexiones
