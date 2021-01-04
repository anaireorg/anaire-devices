# Medidor ANAIRE: CO2 (NDIR), temperatura y humedad
*anaireorg/anaire-devices is licensed under the GNU General Public License v3.0*

*@Asociación Anaire www.anaire.org anaire@anaire.org* 

Fabricación de un medidor de CO2, temperatura y humedad, de mínimo coste y alta precisión, conectado por WiFi a una aplicación en la nube denominada Anaire Cloud App para mostrar los valores instantáneos y el histórico de mediciones de CO2, temperatura y humedad, facilitando así el análisis de la información de las medidas y su correlación con protocolos de ventilación anti COVID-19.

<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxFrontal2.jpg" width="40%" height="40%" />
</p>  
 

# Tabla de contenidos
1. [ Medición de CO2 con sensores NDIR, y envío de las medidas a la nube para su monitorización contínua ](#1)
2. [ Características ](#2)
3. [ Hardware ](#3)
4. [ Software ](#4)
5. [ Fabricación ](#5)
5.1 [ Anaire30ppm ](#5.1)
5.2 [ Anaire50ppm ](#5.2)
6. [ Configuración ](#6)
7. [ Uso ](#7)

---
  
<a name="1"></a>
# 1. Medición de CO2 con sensores NDIR, y envío de las medidas a la nube para su monitorización contínua

El concepto se muestra en la siguiente imagen. Se trata de un dispositivo basado en un microcontrolador ESP8266, al que se conectan sensores para la medida de CO2, temperatura y humedad. El dispositivo tiene un display para mostrar las medidas e indicaciones de estado, un LED y un zumbador para poder emitir alertas visuales y sonoras, y un botón para poder deshabilitar la alerta local. Dispone de conectividad WiFi para poder enviar las mediciones realizadas a una aplicación en la nube, Anaire Cloud (https://github.com/anaireorg/anaire-cloud), que permite agregar la información de múltiples dispositivos y acceder al histórico de medidas desde cualquier dispositivo vía Internet.  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Dispositivo_ANAIRE.png" width="90%" height="90%"/>
</p>
  
En este repositorio se publica información completa para la fabricación del medidor de CO2 Anaire, desde el código fuente hasta las especificaciones de componentes, cableados y opciones de montaje tanto en placa de prototipado como utilizando cajas de impresión 3D para la disposición de los componentes. La intención es que cualquier persona pueda encontrar aquí la información necesaria y suficiente para montar su propio sensor de CO2, con o sin conexión a la nube, sin necesidad de grandes conocimientos técnicos, como ayuda para combatir la pandemia causada por la COVID-19, ayudando a preparar entornos más seguros, con mínimos niveles de CO2, que puedan ayudar a prevenir contagios en entornos cerrados.
  
Toda la información se publica en formato "open source", tanto diseño hardware como código software, con el ánimo de facilitar el uso y difusión de la tecnología todo lo posible. Con el mismo espíritu se tratan de resolver todas las incidencias reportadas en los repositorios de Anaire (*issues*).
  
---  
  
<a name="2"></a>
# 2. Características

**Versión actual: 21.01 ladinadaina (enero de 2021)**

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
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Anaire30ppm.jpg" width="30%" height="30%" />
</p>  
  
* Alternativa para la fabricación mucho más robusta utilizando cajas diseñadas a medida y fabricadas mediante impresoras 3D, y simplemente conectando los componentes mediante cables dupont hembra y encajándolos en los espacios preparados para ello en la caja. La caja es compatible para los dos sensores, y alberga adicionalmente el display OLED (y el resto de los componentes) de forma que sea muy fácil la lectura de las medidas. Los enlaces para la fabricación de la caja se pueden encontrar más adelante.  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxFrontal.jpg" width="30%" height="30%" />
</p>  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxInterior.jpg" width="30%" height="30%" />
</p>  

* Alimentación a través del puerto Micro USB de la tarjeta de control NodeMCU LUA Amica V2.   
  
* Actualización remota de parámetros (umbrales de aviso y de alarma, aviso local de alarma, etc.) desde la aplicación en la nube.    
  
* Portal cautivo para la configuración de la red WiFi en la localización final, sin necesidad de modificar el software del dispositivo. Accesible mediante botón en el dispositivo.  
  
* Actualización remota del SW del dispositivo, iniciada desde la aplicación en la nube. Permite la actualización automática a la última versión del software en modo binario, almacenada en este repositorio en github: https://github.com/anaireorg/anaire-devices/blob/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin    
  
<a name="3"></a>
# 3. Hardware
El medidor Anaire es un dispositivo basado en un microcontrolador ESP8266 dispuesto en una tarjeta de control AZ Delivery NodeMCU Lua Amica V2, que proporciona también conectividad WiFi, y permite su programación desde el IDE de Arduino. Para realizar las medidas se conectan sensores de CO2, temperatura y humedad. Adicionalmente se conectan un display para mostrar las mediciones e indicaciones de estado, y un zumbador para poder emitir alertas sonoras. 
  
Con objeto de simplificar la fabricación y no añadir más componentes aun proporcionando máxima funcionalidad, se utilizan los siguientes elementos ya disponibles en la tarjeta NodeMCU:

<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/NodeMCUV2botonesyleds.png" width="30%" height="30%" />
</p> 

* Se emplea uno de los dos LED incorporados a la tarjeta NodeMCU, el más próximo al conector Micro USB, para proporcionar alertas visuales sobre el estado de CO2. El LED está apagado normalmente; parpadea lentamente cuando el dispositivo está en estado de aviso por CO2, y parpadea rápidamente en caso de encontrarse en alarma. Las frecuencias de parpadeo son iguales a las de la alerta sonora proporcionada por el zumbador.

* Se emplea el botón de Flash (a la derecha del conector Micro USB) poder deshabilitar la alerta local. Y para volverla a habilitar, ya que el botón conmuta entre ambos estados.
  * Adicionalmente, cada vez que se presiona el botón de Flash se muestran el modelo, el ID y la dirección IP del dispositivo, hasta la realización de la siguiente medición.

* Presionando dos veces consecutivas el botón de Reset (a la izquierda del conector MicroUSB) el dispositivo se reinicia en modo de portal cautivo, lo que permite la configuración de la red WiFi y el acceso a otros parámetros de configuración, como se explica en el apartado de Configuración, más adelante.
  
El dispositivo es plenamente operativo incluso sin el display y sin el zumbador. Para funcionar con mínimo coste y complejidad sólo es necesaria una tarjeta de control NodeMCU y un sensor de CO2. Todos los demás elementos son opcionales.

Para los sensores de CO2 existen dos alternativas. Actualmente se soportan dos sensores de CO2, ambos con tecnología NDIR: el **Sensirion SCD30** (https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-co2/) y el **Winsen MH-Z14A** (http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf). 
  
El Sensirion SCD30 es un sensor de mayor calidad: tiene mayor precisión (30 ppm frente a los 50 ppm del MH-Z14A); incorpora sensor de temperatura y humedad, con lo que no hay que añadir un sensor adicional para medir temperatura y humedad, facilitando la fabricación (además de permitir mecanismos de compensación, etc.); tiene un mecanismo de autocalibración más robusto, en el que se hace uso de las mediciones de los últimos 30 días, en lugar de las últimas 24 horas, como en el caso del MH-Z14A; su respuesta es también más rápida ante cambios atmosféricos (por ejemplo, al introducir ventilación en un espacio cerrado). El inconveniente del Sensirion SCD30 es que suele ser más caro y tiene menos opciones de compra y normalmente mayores plazos de entrega. El MH-Z14A está disponible en todo tipo de plataformas de comercio electrónico. Para comprar el Sensirion SCD30 hay que recurrir normalmente a plataformas más especializadas en componentes electrónicos.
  
El software del medidor de CO2 de Anaire es compatible con ambos sensores, y detecta automáticamente cuál de los dos está en uso, adaptándose a ello sin necesidad de realizar ningún cambio de configuración.
  
En caso de utilizar el sensor MH-Z14A como medidor de CO2 se ha incorporado el sensor AZ Delivery DHT11 como sensor de temperatura y humedad. Si se está utilizando el semnsor SCD30 no es necesario incorporar ningún componente adicional, ya que éste incorpora sensor de temperatura y humedad, además de realizar la medida del CO2.
  
**Anaire recomienda utilizar el Sensirion SCD30 siempre que sea posible, ya que permite fabricar, por un coste similar y de forma más sencilla, un dispositivo de mayor precisión y estabilidad en el tiempo**. En cualquier caso se recomienda encarecidamente el análisis de la documentación técnica de ambos sensores, especialmente para la interpretación de las medidas y la determinación de procedimientos de recalibración, en caso de considerarlo necesario para optimizar la evolución de la precisión de las medidas con el paso del tiempo y en función de la utilización del dipositivo (ubicación, régimen de ventilación, etc.)
  
La alimentación del dispositivo se realiza directamente a través del puerto Micro USB de la tarjeta de control NodeMCU LUA Amica V2, el mismo que se utiliza para programarla y para comunicarse con ella. Se recomienda utilizar fuentes de alimentación (enchufes USB, puertos USB en ordenadores, etc.) que puedan proporcionar al menos 500 mA (que es lo más frecuente, aunque podría no ser así en el caso de antiguos cargadores de teléfonos móviles, por ejemplo).   

A continuación se enumeran los elementos citados, incluyendo enlaces a su documentación y a opciones para su adquisición.

## 3.1 Elementos comunes
* Tarjeta de control basada en microcontrolador ESP8266: AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2  
Detalles del producto: https://www.az-delivery.de/es/products/nodemcu  
Disponible en: https://www.amazon.es/dp/B06Y1LZLLY/ref=twister_B082DJVXFC?_encoding=UTF8&psc=1  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/NodeMCU/NodeMCU%20transparente.png" width="25%" height="25%" />  
</p>
  
* Display OLED: AZDelivery 0.91 inch OLED I2C Display 128 x 32 Pixels  
Detalles del producto: https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display  
Disponible en: https://www.amazon.es/dp/B079H2C7WH/ref=twister_B082MC4QJ4?_encoding=UTF8&psc=1  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/Display/Display%20transparente.png" width="25%" height="25%" />  
</p>
  
* Zumbador: AZDelivery Active Buzzer  
Detalles del producto: https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r  
Disponible en: https://www.amazon.es/dp/B089QJKJXW/ref=twister_B082MHYNND?_encoding=UTF8&psc=1  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/Zumbador/Zumbador%20transparente.png" width="10%" height="10%" />  
</p>
  
## 3.2 Sensores
* Medidor Anaire30ppm: sensor de CO2, temperatura y humedad Sensirion SCD30   
Detalles del producto: https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-co2/  
Disponible en: https://www.digikey.es/product-detail/en/sensirion-ag/SCD30/1649-1098-ND/8445334  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/SCD30/SCD30%20transparente.png" width="30%" height="30%" />
</p>  
  
* Medidor Anaire50ppm: sensor de CO2 Winsen MHZ14A  
Detalles del producto: http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf  
Disponible en: https://www.amazon.es/MH-Z14A-di%C3%B3xido-infrarrojo-anal%C3%B3gica-ambiente/dp/B07CXGL7XG  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/MH-Z14A/MH-Z14A%20transparente.png" width="40%" height="40%" />
</p> 
    
  * Adicionalmente, el medidor Anaire50ppm necesita el sensor de temperatura y humedad AZ-Delivery DHT11  
  Detalles del producto: https://www.az-delivery.de/es/products/dht11-temperatursensor-modul  
  Disponible en: https://www.amazon.es/dp/B089W8DB5P/ref=twister_B089YSBB1N?_encoding=UTF8&psc=1  
  <p align="center">
    <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/DHT-11/DHT-11%20transparente.png" width="10%" height="10%" />
  </p>  
  
## 3.3 Otros componentes
 * O bien caja AnaireBox imprimida por 3D  
 Detalles del producto: https://www.thingiverse.com/thing:4694633  
 
 <p align="center">
   <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBox.jpg" width="30%" height="30%" />
 </p>    
   acompañada por cables Dupont hembra-hembra  
   https://www.amazon.es/SODIAL-Puente-Hembra-Soldadura-Flexible/dp/B00HUH9GOC/ref=sr_1_4?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=cable+dupont+hembra+hembra&qid=1609613291&s=industrial&sr=1-4  
 
 * O bien placas de prototipado  
   * Placa de 400 puntos, para el Anaire30ppm  
   Disponible en: https://www.amazon.es/dp/B071ZGC75Y/ref=twister_B07T88TTXF?_encoding=UTF8&psc=1  
   <p align="center">
     <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Breadboard400.png" width="30%" height="30%" />
   </p> 
 
   * Placa de 830 puntos, para el Anaire50ppm  
   Disponible en: https://www.amazon.es/dp/B071ZGC75Y/ref=twister_B07T88TTXF?_encoding=UTF8&psc=1  
   <p align="center">
     <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Breadboard830.png" width="30%" height="30%" />
   </p> 
     
   * En ambos casos son necesarios cables Dupont macho-macho  
   https://www.amazon.es/Neuftech-jumper-20cm-Arduino-Breadboard/dp/B00NBNIETC/ref=sr_1_4?dchild=1&keywords=dupont+macho+macho&qid=1609613744&sr=8-4  
   
 * Será necesario un cable válido para conectar desde un puerto USB del ordenador al puerto Micro USB de la tarjeta NodeMCU, para poder descargar el software del dispositivo. Este mismo cable puede ser utilizado para la alimentación eléctrica desde un ordenador, o desde un enchufe USB o incluso desde una batería portátil USB. También podría utilizarse, una vez descargado el software, un transformador de un antiguo teléfono móvil u otro tipo de dispositivo que tenga un conector Micro USB y proporcione al menos 500 mili amperios. Por ejemplo, el siguiente cable:  
 https://www.amazon.es/TM-Electron-CXU201020-Cable-Blanco/dp/B07BQD6P74/ref=sr_1_22?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=usb+micro+usb&qid=1609613811&sr=8-22  
 O el siguiente alimentador:
 https://www.amazon.es/gp/product/B00U88KSHO/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1  
  
 * Para la conexión de cualquiera de los dos sensores de CO2 es preciso conectar 4 pines y soldarlos en las posiciones indicadas en el proceso de fabricación. Se pueden comprar pines como los siguientes, por ejemplo, y cortar trozos de 4 pines con un alicate de corte o unas tijeras:  
 https://www.amazon.es/Pin-Header-Way-Straight-Pitch/dp/B00QXBRCKG/ref=sr_1_5?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=pin+header&qid=1609617099&s=electronics&sr=1-5  
  
<a name="4"></a>
# 4. Software
El dispositivo se programa exactamente igual que una tarjeta arduino, utilizando el IDE (entorno de desarrollo) de Arduino. Hay que instalar de forma adicional algunos componentes de software, tanto en el PC utilizado como en el propio entorno de Arduino. A continuación se describe en detalle el procedimiento para poder preparar un entorno de desarrollo de SW operativo que permita la descarga del software en los dispositivos (y su programación para modificarlo, en caso deseado).

## 4.1 Preparación del entorno de Arduino
  
 1. Instalar en el ordenador el driver USB-UART (conversión USB a puerto serie) para poder comunicarse con la tarjeta NodeMCU:
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
  
 2. Arrancar el IDE de Arduino, una vez descargado desde el siguiente enlace e instalado en el PC:
https://www.arduino.cc/en/software
  
 3. Abrir la ventana de preferencias (Archivo -> Preferencias)

 4. Para gestionar tarjetas como la NodeMCU, basadas en el microcontrolador ESP8266, introducir en el campo *Gestor de URLs adicionales de tarjetas* lo siguiente: http://arduino.esp8266.com/stable/package_esp8266com_index.json. Se pueden introducir URLs múltiples, separados por comas.

 5. Abrir *Herramientas -> Placa -> Gestor de tarjetas*, localizar *esp8266 platform by esp8266 community* e instalar el software desde la caja de selección

 6. Tras la instalación seleccionar en *Herramientas -> Placa* la opción *NodeMCU 1.0*, para de esta forma trabajar con esa tarjetas

 7. Instalar las siguientes Librerías, utilizando la opción *Herramientas -> Administrar Bibliotecas* del IDE de Arduino. Cuidado: se ha observado que puede fallar si se instalan directamente los archivos zip de las librerías y se descomprimen en el PC local. **Se recomienda encarecidamente instalar las librerías desde el IDE de Arduino, ya que se han observado comportamientos incorrectos al instalar las librerías por otros procedimientos**.
    * WiFiEsp https://github.com/bportaluri/WiFiEsp (uso de WiFi con ESP8266)
    * Arduino Client for MQTT https://pubsubclient.knolleary.net/ (cliente MQTT para comunicación con la nube)
    * ArduinoJson https://arduinojson.org/?utm_source=meta&utm_medium=library.properties (para procesar mensajes MQTT)
    * esp8266-oled-ssd1306 https://github.com/ThingPulse/esp8266-oled-ssd1306 (gestión del display OLED)
    * SparkFun_SCD30_Arduino_Library https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library (gestión del sensor de CO2, temperatura y humedad Sensirion SCD30)
    * DHTesp https://github.com/beegee-tokyo/DHTesp (gestión del sensor de temperatura y humedad DHT11)
    * EspSoftwareSerial https://github.com/plerup/espsoftwareserial/ (comunicación serie con el sensor de CO2 MH-Z14A)
    * ESP_EEPROM https://github.com/jwrw/ESP_EEPROM (almacenamiento de parámetros persistentes en la memoria flash de la NodeMCU)
    * WifiManager kentaylor https://github.com/kentaylor/WiFiManager (gestión de la conexión WiFi)
    * Double Reset detector https://github.com/datacute/DoubleResetDetector (detector de doble pulsación del botón reset para reinicio en el portal cautivo y poder configurar la WiFi y otros parámetros)

## 4.2 Obtención del software del Medidor de CO2 de ANAIRE
El software está disponible en su última versión en el siguiente enlace de este repositorio:  
https://github.com/anaireorg/anaire-devices/blob/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino  

Haga click en el enlace para abrirlo desde el IDE de Arduino.
  
<a name="5"></a>
# 5. Fabricación
Se han diseñado dos modelos de medidores: **Anaire30ppm**, con precisión de 30ppm (Sensirion SCD30) y **Anaire50ppm**, con precisión de 50ppm (sensores Winsen MHZ14A y AZDelivery DHT11). Ambos se pueden montar sobre placa de prototipado o en caja de plástico, utilizando una caja de impresión 3D, para montar mecánicamente los componentes. La caja puede acoger cualquiera de las dos combinaciones de sensores anteriores ya que está diseñada para ser compatible con ambos. Para el caso de montaje sobre placas de prototipado también se ha diseñado una caja a medida sobre la que se puede disponer la placa con los componentes pinchados.

* Caja pequeña AnaireBox para encajar los componentes y cablear mediante cables Dupont hembra-hembra:
https://www.thingiverse.com/thing:4694633  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxFrontal2.jpg" width="40%" height="40%" />
</p>  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxInterior.jpg" width="40%" height="40%" />
</p> 
  
* Caja grande AnaireBread para encajar una placa de prototipado de 830 puntos, con los componentes pinchados y cableados mediante cables Dupont macho-macho:  
https://www.thingiverse.com/thing:4678398  
<p align="center">  
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBread.jpeg" width="40%" height="40%" />
</p>  
<p align="center">  
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBreadAbiertas.jpeg" width="40%" height="40%" />
</p> 
  
Las opciones de montaje que se proponen pretenden simplificar al máximo el proceso, manteniendo las máximas prestaciones técnicas. En cualquier caso se proporciona toda la información necesaria para que cualquiera pueda plantearse alternativas de fabricación, por ejemplo, mediante placas PCB y soldadura de los componentes. Para ello se publican en este mismo repositorio los archivos Fritzing (véase https://fritzing.org/) con la información detallada del cableado requerido:  
  
https://github.com/anaireorg/anaire-devices/blob/main/src/Anaire30ppm_SCD30.fzz  
https://github.com/anaireorg/anaire-devices/blob/main/src/Anaire50ppm_MHZ14A.fzz   
   
<a name="5.1"></a>
## 5.1 Anaire30ppm

### Esquema de conexiones  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/ConexionesAnaire30ppm.png" width="60%" height="60%" />
</p>  
     
### Detalle del sensor SCD30  
Hay que soldar 4 pines como se muestra en la siguiente imagen  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/SCD30/pines_soldados_SCD30.jpg" width="60%" height="60%" />
</p>  
  
### Montaje final  
Montaje sobre breadboard. En este caso es relevante destacar que tras soldar los cuatro pines en el SCD30, utilizando éstos se puede pinchar el componente directamente en la placa de prototipado, alineado correctamente con los pines de la NodeMCU para que se verifique el cableado deseado, ahorrando así cuatro cables y facilitando el engarce mecánico del conjunto de una forma sencilla y muy efectiva.  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/MontajeAnaire30ppm.jpg" width="60%" height="60%" />
</p>  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Anaire30ppm.jpg" width="60%" height="60%" />
</p>  

Montaje en caja AnaireBox:  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Anaire30ppmBox.jpg" width="40%" height="40%" />
</p>  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxFrontal.jpg" width="40%" height="40%" />
</p>  
  
<a name="5.2"></a>
## 5.2 Anaire50ppm

### Esquema de conexiones  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/ConexionesAnaire50ppm.png" width="60%" height="60%" />
</p>  
   
### Detalle del sensor MH-Z14A  
Hay que soldar 4 pines como se muestra en la siguiente imagen  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/MH-Z14A/Pines_CO2_ANAIRE.png" width="60%" height="60%" />
</p>  
  
### Montaje final  
Montaje sobre breadboard:  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/MontajeAnaire50ppm.jpg" width="50%" height="50%" />
</p>  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Anaire50ppm.jpg" width="60%" height="60%" />
</p>  
  
Montaje sobre breadboard en caja AnaireBread:  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Anaire50ppmBreadBox.jpg" width="40%" height="40%" />
</p>

Montaje en caja AnaireBox:  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/Anaire50ppmBox.jpg" width="50%" height="50%" />
</p>  
  
<p align="center">
  <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/AnaireBoxFrontal.jpg" width="40%" height="40%" />
</p>  
  
## 5.3 Carga del software
Una vez completado el montaje de los componentes, hay que proceder a la descarga del software en la tarjeta de control NodeMCU. El software es el mismo, independientemente del modelo de medidor que se haya fabricado. Para ello realice los siguientes pasos:

* Cargue el programa en el IDE de Arduino como se ha indicado previamente en el apartado de *Software*
* Conecte mediante un cable USB el dispositivo al ordenador en el que haya configurado el IDE de Arduino
* Compruebe que la tarjeta se ha detectado y se ha seleccionado el puerto apropiado en *Herramientas -> Puerto*
* Verifique que las variables de conexión a la aplicación de la nube son correctas, en las siguientes líneas al inicio del programa. Por defecto se indican los valores de la aplicación de demo de Anaire:
  > String cloud_server_address = "demo.anaire.org";     // server public IP address  
  > int cloud_app_port = 30183;                          // cloud application port   
* Presione el botón de descarga, que se indica en el IDE con una flecha apuntando hacia la derecha
* Espere varios minutos. Primero se realiza la compilación, luego la descarga del binario compilado en la tarjeta. Durante la fase de descarga debe sonar el pitido del zumbador, por diseño de la aplicación (se ha utilizado a propósito para el zumbador uno de los pines que se emplea para la descarga de software en la tarjeta)
* Compruebe en el display que el dispositivo se ha reiniciado al término del pitido del zumbador
  
<a name="6"></a>
# 6. Configuración
* El dispositivo tiene un identificador único (o ID) generado a partir de la dirección MAC de la interfaz WiFi de cada tarjeta Nodemcu. Este ID es un texto hexadecimal de 6 caracteres, utilizado para el acceso a las medidas en Internet y la integración del medidor en la aplicación en la nube de Anaire. No es preciso realizar ninguna configuración sobre este identificador, se realiza de forma automática.
  * El ID del dispositivo se muestra durante el inicio y tras pulsar el botón de Flash (en la caja AnaireBox es el botón superior, con la letra *A*).
* Configuración de la conexión WiFi
   * Tras pulsar 2 veces consecutivas el botón de Reset (en la caja AnaireBox es el botón superior, con la letra *R*), el dispositivo se reinicia en modo de configuración:
   * Se crea una red WiFi abierta con el nombre ESP_XXXXXX, donde XXXXXX es el identificador del dispositivo
   * Conecte a esa red con un PC, tablet o teléfono móvil
   * Una vez conectado a la red introduzca la siguiente drección IP en su navegador: 192.168.4.1
   * De esta forma se accede al portal de configuración del dispositivo
   * En el portal puede escoger una red WiFi entre las detectadas, e introducir la contraseña de la red. Pulse en el botón *Save* tras escribir la contraseña.
     * Si todo ha ido correctamente el dispositivo se reiniciará con conexión a la nueva red WiFi
     * Si ha habido algún problema de conexión (por ejemplo, si se ha introducido una contraseña errónea), el dispositivo mostrará un mensaje de error WiFi en el display, y deberá repetir el procedimiento
   * En versiones posteriores del software está previsto que en este portal se puedan modificar otros parámetros de configuración, como la conexión a la aplicación en la nube o los umbrales de aviso y alerta CO2. Actualmente se puede configurar únicamente la conexión WiFi.
  
<a name="7"></a>
# 7. Uso
* Alimentar el dispositivo con un cable conectado al conector micro usb   
* Durante los primeros 3 segundos debería ver el texto *anaire.org* en el display
  * A continuación se mostrarán tres líneas en el display, indicando el modelo de dispositivo, el ID automáticamente asignado y un número con una cuenta atrás por segundos debida al precalentamiento del sensor de CO2 (1 minuto en el Anaire30ppm, 3 minutos en el Anaire50ppm). Los sensores de CO2 requieren de un tiempo de estabilización y calentamiento para su puesta en marcha de forma correcta.  
  * Una vez concluida la cuenta atrás el dispositivo mostrará las medidas de CO2, temperatura y humedad en la primera línea, y una indicación del estado del CO2 (*CO2 BIEN*, *CO2 REGULAR* o *CO2 MAL*) en la segunda
    * En caso de que haya algún error el estado de CO2 será sustituido por un mensaje indicando el tipo de error experimentado
* En caso de que el valor medido de CO2 supere el umbral de aviso, el dispositivo empezará a emitir un pitido intermitente, así como el parpadeo del LED de estado de CO2, de forma alternativa
  * Los valores por defecto de los umbrales son 700 PPM para el aviso y 1.000 PPM para la alarma. Estos valores se pueden modificar desde la aplicación de Anaire en la nube.
  * En caso de que el valor medido de CO2 supero el umbral de alarma, el dispositivo empezará a emitir un pitido intermitente, así como el parpadeo del LED de estado de CO2, de forma alternativas y con una frecuencia mayor que en el caso de aviso
* Si se desea detener la indicación local de alarma (visual y sonora), presione una vez el botón de Flash (a la derecha del conector USB)
  * El display mostrará, de forma adicional, el modelo, el ID y la dirección IP del dispositivo
  * Si se vuelve a presional el botón de Flash, se reactivará el aviso de alarma local. Es decir, el botón de Flash permite conmutar entre avisar o no de forma local cuando los valores de CO2 superen los umbrales estabilizaciónecidos
  * Cuando la medida de CO2 sea inferior al umbral de aviso, el dispositivo reseteará el estado de la señal local de alarma, de forma que automáticamente volverá a dar indicaciones locales de alarma si se vuelven a superar los umbrales de aviso o de alarma en el futuro, sin necesidad de reactivar la alarma local
* Si hay algún error, la última línea del display mostrará el error, en lugar de dar una indicación del estado de CO2. El dispositivo está diseñado para recuperarse automáticamente de los errores cuando la causa que los provoca se ha resuelto, sin que el usuario tenga que intervenir.
* Si se conecta con un dispositivo a la misma red WiFi a la que esta conectado el medidor, accediendo a la dirección IP del medidor se pueden ver las últimas medidas realizadas y los umbrales de aviso y alarma CO2 definidos.
* En la misma dirección IP del medidor se puede también hacer click en un enlace que realiza la calibración del medidor de CO2. Debe disponer el medidor en un entorno ventilado, preferiblemente al aire libre, y esperar el tiempo indicado por la cuenta atrás en el display una vez iniciado el proceso desde un navegador.
* Acceso por internet a las medidas:
  * Si se han dejado los valores por defecto de la aplicación en la nube, se puede acceder a los valores medidos por el dispositivo en los siguientes enlaces:
    * Valores actuales en modo kiosko (sustituir [ID_del_sensor] por el ID del medidor, y [Nombre_que_se_desea_visualizar] por el nombre deseado):  
    https://demo.anaire.org/sensor/[ID_del_sensor]/[Nombre_que_se_desea_visualizar]
    <p align="center">
      <img src="https://github.com/anaireorg/anaire-devices/blob/main/images/dispositivos/MedidasModoKiosko.jpg" width="30%" height="30%" />
    </p>  
    * Valores actuales con acceso a histórico (sustituir [ID_del_sensor] por el ID del medidor, y [Nombre_que_se_desea_visualizar] por el nombre deseado):  
    https://demo.anaire.org/detalle/[ID_del_sensor]/[Nombre_que_se_desea_visualizar]  

## 7.1 Errores mostrados en el display
* err_wifi: no se pudo conectar a la red WiFi. Compruebe el estado de la conexión presionando dos veces consecutivas el botón de Reset y conectando al portal cautivo, como se explica en el apartado de Configuración del dispositivo.
* err_mqtt: no se pudo conectar al endpoint de la aplicación de Anaire en la nube. Compruebe los detalles al inicio del código del dispositivo y verifique la conectividad del endpoint definido.
* err_co2: no se pudo conectar con el sensor de CO2. Compruebe las conexiones.
* err_dht: no se pudo conectar con el sensor de humedad y temperatura DHT11. Compruebe las conexiones.
  
## 7.2 Diagnóstico de problemas
Conecte el dispositivo al PC utilizando un cable USB. Arranque el monitor serie del IDE de Arduino con la opción *Herramientas -> Monitor serie*. Se abrirá una nueva ventana en la que se imprimirán todos los mensajes emitidos por el dispositivo durante su funcionamiento, que ayudarán a diagnosticar los posibles problemas. Se recomienda presionar una vez el botón de *Reset* para reiniciar el dispositivo y poder observar un ciclo completo de funcionamiento desde el principio.
  
