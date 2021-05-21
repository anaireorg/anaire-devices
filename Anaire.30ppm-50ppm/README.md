# Anaire 30/50ppm

*@2021 Anaire www.anaire.org*

---
<p align="center">
  <img src="docs\AnaireBoxFrontal2.jpg" width="40%" height="40%" />
</p>  

# Tabla de contenidos
[1. Hardware](#id1)  
[2. Software](#id2)  
[3. Fabricación](#id3)  
[4. Configuración](#id4)  
[5. Uso](#id5)  
[6. Diagnóstico](#id6)  

---
<a name="id1"></a>

# 1. Hardware
El medidor Anaire es un dispositivo basado en un microcontrolador ESP8266 dispuesto en una tarjeta de control [AZ Delivery NodeMCU LUA Amica V2](https://www.az-delivery.de/es/products/nodemcu?_pos=3&_sid=fd8b40ecd&_ss=r), que proporciona también conectividad WiFi, y permite su programación desde el IDE de Arduino. Para realizar las medidas se conectan sensores de CO2, temperatura y humedad. Adicionalmente se conectan un display para mostrar las mediciones e indicaciones de estado, y un zumbador para poder emitir alertas sonoras.

Con objeto de simplificar la fabricación y no añadir más componentes aun proporcionando máxima funcionalidad, se utilizan los siguientes elementos ya disponibles en la tarjeta NodeMCU:

<p align="center">
  <img src="docs\NodeMCUV2botonesyleds.png" width="30%" height="30%" />
</p>

* Se emplea uno de los dos LED incorporados a la tarjeta NodeMCU, el más próximo al conector Micro USB, para proporcionar alertas visuales sobre el estado de CO2. El LED está encendido en estado normal, es decir, cuando el valor medido del CO2 está por debajo del umbral de aviso; parpadea lentamente cuando el dispositivo está en estado de aviso por CO2, y parpadea rápidamente en caso de encontrarse en alarma. Las frecuencias de parpadeo son iguales a las de la alerta sonora proporcionada por el zumbador.

* Se emplea el botón de Flash (a la derecha del conector Micro USB) poder deshabilitar la alerta local. Y para volverla a habilitar, ya que el botón conmuta entre ambos estados.
  * Adicionalmente, cada vez que se presiona el botón de Flash se muestran el modelo, el ID y la dirección IP del dispositivo, hasta la realización de la siguiente medición.

* Presionando dos veces consecutivas el botón de Reset (a la izquierda del conector MicroUSB) el dispositivo se reinicia en modo de portal cautivo, lo que permite la configuración de la red WiFi y el acceso a otros parámetros de configuración, como se explica en el apartado de Configuración, más adelante.

El dispositivo es plenamente operativo incluso sin el display y sin el zumbador. Para funcionar con mínimo coste y complejidad sólo es necesaria una tarjeta de control NodeMCU y un sensor de CO2. Todos los demás elementos son opcionales.

Para los sensores de CO2 existen dos alternativas. Actualmente se soportan dos sensores de CO2, ambos con tecnología NDIR: el [*Sensirion SCD30*](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-scd30/) y el [*Winsen MHZ14A*](https://www.winsen-sensor.com/sensors/co2-sensor/mh-z14a.html).

El Sensirion SCD30 es un sensor de mayor calidad: tiene mayor precisión (30 ppm frente a los 50 ppm del MH-Z14A); incorpora sensor de temperatura y humedad, con lo que no hay que añadir un sensor adicional para medir temperatura y humedad, facilitando la fabricación, además de permitir mecanismos de compensación por altitud y temperatura; tiene un mecanismo de autocalibración más robusto, en el que se hace uso de las mediciones de los últimos 30 días, en lugar de las últimas 24 horas, como en el caso del MH-Z14A; su respuesta es también más rápida ante cambios atmosféricos (por ejemplo, al introducir ventilación en un espacio cerrado). Los inconvenientes del Sensirion SCD30 son que es más caro, que es más frágil para su manipulación y que tiene menos opciones de compra y normalmente mayores plazos de entrega. El MH-Z14A está disponible en todo tipo de plataformas de comercio electrónico. Para comprar el Sensirion SCD30 hay que recurrir normalmente a plataformas más especializadas en componentes electrónicos.

El software del medidor Anaire 30/50ppm es compatible con ambos sensores, y detecta automáticamente cuál de los dos está en uso, adaptándose a ello sin necesidad de realizar ningún cambio de configuración.

En caso de utilizar el sensor MH-Z14A como medidor de CO2 se ha incorporado adicionalmente el sensor [AZ Delivery DHT11](https://www.az-delivery.de/es/products/dht11-temperatursensor-modul) como sensor de temperatura y humedad. Si se está utilizando el sensor SCD30 no es necesario incorporar ningún componente adicional, ya que éste incorpora sensor de temperatura y humedad, además de realizar la medida del CO2. En cualquier caso se recomienda encarecidamente el análisis de la documentación técnica de ambos sensores, especialmente para la interpretación de las medidas y la determinación de procedimientos de recalibración, en caso de considerarlo necesario para optimizar la evolución de la precisión de las medidas con el paso del tiempo y en función de la utilización del dipositivo (ubicación, régimen de ventilación, etc.)

La alimentación del dispositivo se realiza directamente a través del puerto Micro USB de la tarjeta de control [NodeMCU LUA Amica V2](https://www.az-delivery.de/es/products/nodemcu?_pos=3&_sid=fd8b40ecd&_ss=r), el mismo que se utiliza para programarla y para comunicarse con ella. Se recomienda utilizar fuentes de alimentación (enchufes USB, puertos USB en ordenadores, etc.) que puedan proporcionar al menos 1.000 mA (que es lo más frecuente, aunque podría no ser así en el caso de antiguos cargadores de teléfonos móviles, por ejemplo).   

A continuación se enumeran los elementos citados, incluyendo enlaces a su documentación.

## 1.1 Elementos comunes
* Tarjeta de control basada en microcontrolador ESP8266:  [AZDelivery NodeMCU LUA Amica V2](https://www.az-delivery.de/es/products/nodemcu?_pos=3&_sid=fd8b40ecd&_ss=r)  

<p align="center">
  <img src="docs\NodeMCU transparente.png" width="35%" height="35%" />  
</p>

* Display OLED: [AZDelivery 0.91 inch OLED I2C Display 128 x 32 Pixels](https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display?_pos=2&_sid=8c7ab3c5f&_ss=r)  

<p align="center">
  <img src="docs\Display transparente.png" width="35%" height="35%" />  
</p>

* Zumbador: [AZDelivery Active Buzzer](https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2)  

<p align="center">
  <img src="docs\Zumbador transparente.png" width="20%" height="20%" />  
</p>

* Caja [AnaireBox](https://www.thingiverse.com/thing:4694633) para impresora 3D  

<p align="center">
  <img src="docs\AnaireBoxFrontal2.jpg" width="40%" height="40%" />
</p>    
<p align="center">
  <img src="docs\AnaireBoxInterior.jpg" width="50%" height="50%" />
</p>  

* [Cables Dupont hembra-hembra de 10cm de longitud](https://www.amazon.es/SODIAL-Puente-Hembra-Soldadura-Flexible/dp/B00HUH9GOC/ref=sr_1_4?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=cable+dupont+hembra+hembra&qid=1609613291&s=industrial&sr=1-4)

<p align="center">
  <img src="docs\Cables Dupont.png" width="40%" height="40%" />
</p>

* Será necesario un cable válido para conectar desde un puerto USB del ordenador al puerto Micro USB de la tarjeta NodeMCU, para poder cargar el software del dispositivo. Este mismo cable puede ser utilizado para la alimentación eléctrica desde un ordenador, o desde un enchufe USB o incluso desde una batería portátil USB. También podría utilizarse, una vez cargado el software, un transformador de un antiguo teléfono móvil u otro tipo de dispositivo que tenga un conector Micro USB y proporcione al menos 1.000 miliamperios. Por ejemplo, [este cable](https://www.amazon.es/TM-Electron-CXU201020-Cable-Blanco/dp/B07BQD6P74/ref=sr_1_22?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=usb+micro+usb&qid=1609613811&sr=8-22), o [este alimentador](https://www.amazon.es/gp/product/B00U88KSHO/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)

## 1.2 Sensores
* Anaire30ppm: [*Sensirion SCD30*](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-scd30/)
<p align="center">
  <img src="docs\SCD30 transparente.png" width="40%" height="40%" />
</p>  

* Anaire50ppm: [*Winsen MHZ14A*](https://www.winsen-sensor.com/sensors/co2-sensor/mh-z14a.html)  
<p align="center">
  <img src="docs\MH-Z14A transparente.png" width="60%" height="60%" />
</p>

* Adicionalmente, el medidor Anaire50ppm necesita el sensor de temperatura y humedad [AZ Delivery DHT11](https://www.az-delivery.de/es/products/dht11-temperatursensor-modul)  
  <p align="center">
    <img src="docs\DHT-11 transparente.png" width="20%" height="20%" />
  </p>  

* Para la conexión de cualquiera de los dos sensores de CO2 es preciso conectar 4 pines y soldarlos en las posiciones indicadas en el proceso de fabricación. Se pueden comprar pines como [éstos](https://www.amazon.es/Pin-Header-Way-Straight-Pitch/dp/B00QXBRCKG/ref=sr_1_5?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=pin+header&qid=1609617099&s=electronics&sr=1-5), por ejemplo, y cortar trozos de 4 pines con un alicate de corte o unas tijeras.

<a name="id2"></a>

# 2. Software

El dispositivo se programa exactamente igual que una tarjeta arduino, utilizando el [entorno de desarrollo de Arduino](https://www.arduino.cc/en/software).

Adicionalmente hay que instalar algunos componentes de software en el entorno de Arduino, principalmente diversas librerías. La descripción completa de la instalación del entorno de *software* se puede encontrar al inicio del archivo de código fuente del dispositivo medidor [NodeMCULuaAmicaV2.ino](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino).

En cualquier caso la instalación del entorno completo de desarrollo no será necesaria si no se requiere hacer modificaciones en el software. Para la fabricación del dispositivo se proporciona un mecanismo sencillo basado en la carga del [binario ya compilado](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin) en la tarjeta NodeMCU, como se describe más adelante, en el apartado de fabricación.

Para cualquiera de las dos opciones será necesario instalar previamente en un ordenador con sistema operativo Windows10 el [driver USB-UART](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (conversión USB a puerto serie) para poder comunicarse con la tarjeta TTGO T-Display.

A continuación se describe en detalle el procedimiento para poder preparar un entorno de desarrollo de SW operativo que permita la carga del software en los dispositivos y su programación para modificarlo, en caso deseado. En este vídeo se puede seguir todo el proceso paso a paso tanto para la preparación del entorno como para la carga del sotware.

[![Preparación del entorno y carga del software para Anaire30ppm y Anaire50ppm](http://img.youtube.com/vi/nSNwYwKNmuk/0.jpg)](https://youtu.be/nSNwYwKNmuk)

<a name="id3"></a>

# 3. Fabricación

Se han diseñado dos modelos de medidores basados en el *hardware* descrito: **Anaire30ppm**, con precisión de 30 ppm (sensor Sensirion SCD30) y **Anaire50ppm**, con precisión de 50 ppm (sensores Winsen MHZ14A y AZ Delivery DHT11). Ambos se pueden fabricar mediante la misma caja [AnaireBox](https://www.thingiverse.com/thing:4694633), utilizando una impresora 3D, para montar mecánicamente los componentes. La caja puede acoger cualquiera de las dos combinaciones de sensores anteriores ya que está diseñada para ser compatible con ambos.  para impresora 3D

Las opciones de montaje que se proponen pretenden simplificar al máximo el proceso, manteniendo las máximas prestaciones técnicas. En cualquier caso se proporciona toda la información necesaria para que cualquiera pueda plantearse alternativas de fabricación, por ejemplo, mediante placas PCB y soldadura de los componentes. Para ello se publican en este mismo repositorio los archivos necesarios en formato de la aplicación [Fritzing](https://fritzing.org/) con la información detallada del cableado requerido, que pueden servir de punto de partida para el diseño de la placa PCB.  

  * [Archivo Fritzing para Anaire30ppm (Sensirion SCD-30)](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/docs/Anaire30ppm_SCD30.fzz)

  * [Archivo Fritzing para Anaire50ppm (Winsen MH-Z14A)](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/docs/Anaire50ppm_MHZ14A.fzz)

<a name="3.1"></a>
## 3.1 Anaire30ppm

### Esquema de conexiones  
<p align="center">
  <img src="docs\ANAIRE_30_ppm.png" width="80%" height="80%" />
</p>  

### Detalle del sensor SCD30  
Hay que soldar 4 pines como se muestra en la siguiente imagen  
<p align="center">
  <img src="docs\pines_soldados_SCD30.jpg" width="60%" height="60%" />
</p>  

### Montaje final  

El siguiente vídeo muestra de forma detallada el montaje completo del medidor Anaire 30ppm:

[![Montaje Anaire30ppm box](http://img.youtube.com/vi/VRJ4Hir3wzU/0.jpg)](https://youtu.be/VRJ4Hir3wzU)

<p align="center">
  <img src="docs\Anaire30ppmBox.jpg" width="40%" height="40%" />
</p>  

<p align="center">
  <img src="docs\AnaireBoxFrontal2.jpg" width="40%" height="40%" />
</p>  

## 3.2 Anaire50ppm

### Esquema de conexiones  
<p align="center">
  <img src="docs\ANAIRE_50_ppm.png" width="60%" height="60%" />
</p>  

### Detalle del sensor MH-Z14A  
Hay que soldar 4 pines como se muestra en la siguiente imagen  
<p align="center">
  <img src="docs\Pines_CO2_ANAIRE.png" width="80%" height="80%" />
</p>  

### Montaje final  
El siguiente vídeo muestra el montaje completo del medidor Anaire 50ppm en detalle:

[![Montaje de Anaire50ppm Box](http://img.youtube.com/vi/fMnoskmz7p4/0.jpg)](https://youtu.be/fMnoskmz7p4)

Imágenes del montaje:
<p align="center">
  <img src="docs\Anaire50ppmBox.jpg" width="50%" height="50%" />
</p>  

<p align="center">
  <img src="docs\AnaireBoxFrontal.jpg" width="50%" height="50%" />
</p>  

## 3.3 Carga del software
Una vez completado el montaje de los componentes, hay que proceder a la carga del software en la tarjeta de control NodeMCU. El software es el mismo, independientemente del modelo de medidor que se haya fabricado.

Para la carga del software hay dos opciones: carga del binario compilado, o carga del código fuente desde el IDE de Arduino. Como ya se ha explicado anteriormente, si no se desea modificar el software se recomienda la primera opción, ya que es mucho más sencilla y rápida.

En cualquiera de los dos casos es preciso instalar previamente en un ordenador con sistema operativo Windows 10 el [driver USB-UART](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (conversión USB a puerto serie) para poder comunicarse con la tarjeta TTGO T-Display.

### Carga del binario compilado
Estas instrucciones son únicamente válidas para ordenadores con el sistema operativo Windows 10.

Asegure que se tiene en un directorio los siguientes archivos y carpetas, disponibles en la carpeta [anaire-device.NodeMCULuaAmicaV2](https://github.com/anaireorg/anaire-devices/tree/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2):

Archivos:
  * flash_device.bat
  * anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin
Carpeta:
  * flash_device_tools

Ejecute los siguientes pasos en esa carpeta:

1. Conecte el medidor Anaire 30/50ppm al ordenador mediante un cable MicroUSB. En el administrador de dispositivos de windows podrá identificar el puerto COM asignado por el sistema operativo al medidor Anaire 30/50ppm.

<p align="center">
  <img src="docs\Identificación de puerto COM.png" width="80%" height="80%" />
</p>

2. Haga click derecho en el archivo [flash_device.bat](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/flash_device.bat) y seleccione la opción de *editar*

3. Donde pone *--port COMXX* sustituya el puerto COMXX para poner el que haya identificado previamente en el paso 1, seleccione *archivo->guardar* y cierre la ventana

<p align="center">
  <img src="docs\Edición de puerto COM.png" width="100%" height="100%" />
</p>

4. Haga doble click en el archivo [flash_device.bat](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/flash_device.bat). Al acabar la ejecución pedirá que se pulse cualquier tecla para cerrar la ventana. El medidor Anaire 30/50ppm arrancará con la última versión de *software* disponible en el archivo [anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin)

<p align="center">
  <img src="docs\Ejecución carga de binario.png" width="100%" height="100%" />
</p>

### Carga desde el código fuente

1. Instale el [entorno de desarrollo de Arduino](https://www.arduino.cc/en/software).

2. Instale los componentes de software requeridos en el entorno de Arduino, tal como se describe al inicio del archivo de código fuente del dispositivo medidor [anaire-device.NodeMCULuaAmicaV2.ino](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino).

3. Cargue el programa [anaire-device.NodeMCULuaAmicaV2.ino](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino) en el IDE de Arduino utilizando la opción *Archivo->Abrir*

4. Conecte mediante un cable MicroUSB el Anaire 30/50ppm al ordenador

5. Seleccione el puerto apropiado en *Herramientas -> Puerto*. Si sólo tiene un Anaire 30/50ppm conectado, sólo debería aparecer una opción para seleccionar. Si no aparece ninguna, o bien no se ha instalado correctamente el [driver USB-UART](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers), o bien el cable MicroUSB o la tarjeta NodeMCU son defectuosos.

6. Presione el botón de carga, que se indica en el IDE con una flecha apuntando hacia la derecha, y espere a que se compile y se descargue el software en la tarjeta NodeMCU.

<p align="center">
  <img src="docs\Descarga de SW desde el IDE de Arduino.png" width="100%" height="100%" />
</p>

El proceso puede durar varios minutos, dependiendo de la potencia de cálculo del ordenador. Al término del proceso en el IDE de Arduino se mostrará un mensaje indicando que se ha escrito el 100% del programa en la tarjeta, y el dispositivo PiCO2 se reiniciará mostrando durante varios segundos la imagen de Anaire en la pantalla, para a continuación comenzar a mostrar medidas cada 5 segundos.

En el siguiente vídeo puede verse el proceso completo al detalle:
[![Preparación del entorno y carga del software para Anaire30ppm y Anaire50ppm](http://img.youtube.com/vi/nSNwYwKNmuk/0.jpg)](https://youtu.be/nSNwYwKNmuk?t=260)

<a name="id4"></a>

# 4. Configuración

El medidor tiene un identificador único (o ID) generado a partir de la dirección MAC de la interfaz WiFi de cada tarjeta NodeMCULuaAmicaV2. Este ID es un texto hexadecimal de 6 caracteres, utilizado para el acceso a las medidas en Internet y la integración del medidor en la aplicación en la nube de Anaire. No es preciso realizar ninguna configuración sobre este identificador, se realiza de forma automática.

El ID del dispositivo se muestra durante el inicio y tras pulsar el botón de Flash (en la caja AnaireBox es el botón superior, con la letra *A*).

Para completar la configuración del dispositivo Anaire 30/50ppm hay que conectarlo a una red WiFi y realizar la calibración de referencia.

## Configuración de la conexión WiFi

* Tras pulsar 2 veces consecutivas el botón de Reset (en la caja AnaireBox es el botón inferior, con la letra *R* o con la letra *B*), el dispositivo se reinicia en modo de configuración, creando una red WiFi y un portal cautivo para la configuración:
   * Se crea una red WiFi abierta con el nombre ESP_XXXXXX, donde XXXXXX es el identificador del dispositivo
   * Conecte a esa red con un PC, tablet o teléfono móvil
   * Una vez conectado a la red introduzca la siguiente dirección IP **192.168.4.1** en su navegador. De esta forma se accede al portal de configuración del dispositivo
   * En el portal puede escoger una red WiFi entre las detectadas, e introducir la contraseña de la red. Pulse en el botón *Save* tras escribir la contraseña.
     * Si todo ha ido correctamente el dispositivo se reiniciará con conexión a la nueva red WiFi
     * Si ha habido algún problema de conexión (por ejemplo, si se ha introducido una contraseña errónea), el dispositivo mostrará un mensaje de error WiFi en el display, y deberá repetir el procedimiento

## Calibración de referencia

Sitúe el medidor Anaire 30/50ppm al aire libre, protegido del sol, de otras fuentes de luz intensas y de las corrientes de aire (por ejemplo, dentro de una caja de cartón en el alféizar de la ventana), y arranque el proceso de calibración mediante alguno de los siguientes métodos:
 - Presione el botón superior, etiquetado con la letra "A" durante más de 10 segundos, hasta que la pantalla muestre una cuenta atrás indicando el número de segundos restantes hasta que se complete la calibración.
 - Identifique la dirección IP del dispositivo pulsando brevemente el botón superior, e introduzca la dirección en un navegador de un dispositivo conectado a la misma red WiFi. En el servidor web haga click en el enlace que permite iniciar la calibración de referencia del dispositivo.
 - Acceda a la aplicación en la nube, entrando en la opción de editor de parámetros. Para ello debe tener permisos de administrador en la aplicación. Active la opción denominada *Forced calibration* y haga click en el botón *Update*. de esta forma podrá iniciar la calibración a través de Internet.

 En cualquiera de los métodos anteriores la pantalla del Anaire 30/50ppm una cuenta atrás para indicar el tiempo restante para que se complete la calibración forzada. Este tiempo depende del sensor equipado, siendo de 1 minuto en el caso del Sensirion SCD30 y de 10 minutos en el caso del Winsen MHZ-14A. No interfiera con el dispositivo, ni lo manipule o lo acerque a fuentes de CO2 (como la respiración) mientras dure la cuenta atrás.

 Al término del proceso el dispositivo se reiniciará y volverá a tomar medidas, tomando como referencia el valor de 400 ppm (este valor se puede modificar desde la aplicación en la nube para el sensor scd30). El proceso no puede ser interrumpido, deberá esperar a su término en cualquier caso, o retirar la alimentación del Anaire 30/50ppm si desea interrumpirlo.

<a name="id5"></a>

# 5. Uso

Para empezar a realizar medidas sólo tiene que alimentar el Anaire 30/50ppm con un cable MicroUSB.
* Durante un segundo se muestra el texto *anaire.org* en el display
* A continuación el ID y la versión de *software* durante otro segundo
* A continuación se mostrarán tres líneas en el display, indicando el modelo de dispositivo (anaire 30ppm o 50ppm), el ID automáticamente asignado y un número con una cuenta atrás por segundos debida al precalentamiento del sensor de CO2: 3 segundos en el Anaire30ppm, 3 minutos en el Anaire50ppm. Los sensores de CO2 requieren de un tiempo de estabilización y calentamiento para su puesta en marcha de forma correcta.  
* Una vez concluida la cuenta atrás el dispositivo mostrará las medidas de CO2, temperatura y humedad.

En caso de que el valor medido de CO2 supere el umbral de aviso, el dispositivo empezará a emitir un pitido intermitente, así como el parpadeo del LED de estado de CO2 y del valor en el display.Los valores por defecto de los umbrales son 700 PPM para el aviso y 1.000 PPM para la alarma. Estos valores se pueden modificar desde la aplicación de Anaire en la nube.

Si se desea detener la indicación local de alarma (visual y sonora), presione una vez el botón "A" ("flash" en la propia placa). El display mostrará, de forma adicional, el modelo, el ID y la dirección IP del dispositivo. Si se vuelve a presional el botón de Flash, se reactivará el aviso de alarma local. Es decir, el botón de Flash permite conmutar entre avisar o no de forma local cuando los valores de CO2 superen los umbrales establecidos.

Cuando la medida de CO2 sea inferior al umbral de aviso, el dispositivo reseteará el estado de la señal local de alarma, de forma que automáticamente volverá a dar indicaciones locales de alarma si se vuelven a superar los umbrales de aviso o de alarma en el futuro, sin necesidad de reactivar la alarma local.

Si hay algún error, la última línea del display mostrará el error. El dispositivo está diseñado para recuperarse automáticamente de los errores cuando la causa que los provoca se ha resuelto, sin que el usuario tenga que intervenir.

##  Botones
El Anaire 30/50ppm tiene dos botones en la parte inferior derecha, el superior, etiquetado con la letra "A", y el inferior, etiquetado con la letra "R". Puede usar los botones para realizar las siguientes funciones:

### Botón superior "A"
- Pulsación corta - Conmuta el estado de la alarma acústica y muestra información del dispositivo: ID,  dirección IP y versión de *software*
- Pulsación larga durante más de 10 segundos - Inicia el proceso de calibración forzada.

### Botón inferior "R"
- Pulsación corta - Reinicia el Anaire 30/50ppm
- Doble pulsación en menos de 10 segundos - Arranca el portal cautivo que permite configurar la conexión WiFi y la dirección del servidor MQTT de la aplicación Anaire en la nube a la que enviar las medidas

## Servidor Web
Si se conecta con un dispositivo a la misma red WiFi a la que esta conectado el medidor PiCO2, accediendo a la dirección IP del medidor mediante un navegador se puede acceder al servidor web del Anaire 30/50ppm, pudiendo así acceder a información detallada del dispositivo, a las últimas medidas, e incluso pudiendo realizar diversas acciones de configuración, como se puede ver en la siguiente imagen:

<p align="center">
  <img src="docs\Servidor Web.png" width="100%" height="100%" />
</p>

## Acceso por internet a las medidas:
Si se ha conectado el Anaire 30/50ppm a una red WiFi con acceso a internet y se han dejado los valores por defecto de conexión a la aplicación en la nube, se puede acceder a los valores medidos por el dispositivo en los siguientes enlaces. Sustituya *ID_del_dispositivo* por el ID del PiCO2 en cuestión para acceder a las medidas:

* Valores actuales en modo kiosko (modo muy práctico para visualizar las medidas en un teléfono móvil por ejemplo):  
    https://portal.anaire.org/sensor/ID_del_dispositivo/

    <p align="center">
      <img src="docs\MedidasModoKiosko.jpg" width="30%" height="30%" />
    </p>  

* Valores actuales con acceso a histórico. Pulsando en el botón de *Detalle* podrá visualizar la evolución temporal de las medidas.
    https://portal.anaire.org/admin/ID_del_dispositivo/

    <p align="center">
      <img src="docs\Medidas detalle 1.png" width="100%" height="100%" />
    </p>

    <p align="center">
      <img src="docs\Medidas detalle 2.png" width="100%" height="100%" />
    </p>

Si desea gestionar un conjunto de dispositivos PiCO2 o Anaire 30/50ppm para una organización, envíe los ID de sus dispositivos al correo anaire@anaire.org para que le configuremos cuadros de control automáticos como los siguientes, sin coste alguno:

Panel general con la visión instantánea:
<p align="center">
  <img src="docs\Panel General.png" width="100%" height="100%" />
</p>

Panel de evolución temporal:
<p align="center">
  <img src="docs\Panel Detalle.png" width="100%" height="100%" />
</p>

También podemos configurar alarmas por correo electrónico, por Telegram, etc. Póngase en contacto con nosotros si tiene necesidades de gestión de un conjunto de dispositivos para que podamos ayudarle a diseñar una solución a medida de su organización.

<a name="id6"></a>

# 6. Diagnóstico

Conecte el dispositivo al PC utilizando un cable USB. Arranque el monitor serie del IDE de Arduino con la opción *Herramientas -> Monitor serie*. Se abrirá una nueva ventana en la que se imprimirán todos los mensajes emitidos por el dispositivo durante su funcionamiento, que ayudarán a diagnosticar los posibles problemas. Se recomienda presionar una vez el botón de *Reset* para reiniciar el dispositivo y poder observar un ciclo completo de funcionamiento desde el principio.

Los siguientes errores se pueden mostrar en el display del medidor. Se indica su significado y cómo proceder para cada uno de ellos:

* err_wifi: no se pudo conectar a la red WiFi. Compruebe el estado de la conexión presionando dos veces consecutivas el botón de *Reset* y conectando al portal cautivo, como se explica en el apartado de Configuración del dispositivo.
* err_mqtt: no se pudo conectar al servidor MQTT de la aplicación de Anaire en la nube. Compruebe los detalles al inicio del código del dispositivo y verifique la conectividad del servidor definido.
* err_co2: no se pudo conectar con el sensor de CO2. Compruebe las conexiones.
* err_dht: no se pudo conectar con el sensor de humedad y temperatura DHT11. Compruebe las conexiones.

---

# BONUS

## Montaje sobre Breadboard

El montaje sobre breadboard es algo que puede ser útil para proyectos educativos, o para desarrollo, o incluso en caso de no tener acceso a una impresora 3D.

***Anaire 30ppm***

Es extremadamente sencillo montar el Anaire30ppm sobre breadboard. El siguiente vídeo muestra el proceso en detalle:

[![Montaje Anaire30ppm breadboard](http://img.youtube.com/vi/d87cvSwYpxk/0.jpg)](https://youtu.be/d87cvSwYpxk)

En este caso es relevante destacar que tras soldar los cuatro pines en el SCD30, utilizando éstos se puede pinchar el sensor directamente en la placa de prototipado, alineado correctamente con los pines de la NodeMCU para que se verifique el cableado deseado, ahorrando así cuatro cables y facilitando el engarce mecánico del conjunto de una forma sencilla y muy efectiva.

<p align="center">
  <img src="docs/Anaire30ppm.jpg" width="60%" height="60%" />
</p>  

Este montaje es muy sencillo de realizar, pero hay que tener en cuenta que la proximidad entre el sensor SCD30 y el microcontrolador en la placa NodeMCU puede producir que haya alteración de las medidas realizadas, especialmente las de temperatura y humedad. Adicionalmente, el sensor queda expuesto a corrientes de aire o fuentes de luz que producen alteraciones significativas de las medidas. Siempre resulta conveniento poder fijar mecánicamente el conjunto en algún tipo de caja protectora.

***Anaire 50ppm***

El siguiente vídeo muestra de forma detallada el montaje completo con el sensor MHZ-14A sobre breadboard:

[![Montaje de Anaire50ppm Breadboard](http://img.youtube.com/vi/tY8MLtDSswg/0.jpg)](https://youtu.be/tY8MLtDSswg)

<p align="center">
  <img src="docs\Anaire50ppm.jpg" width="60%" height="60%" />
</p>  

De nuevo es deseable introducir en montaje en una caja que pueda proteger el prototipo de corrientes de aire o la exposición directa a fuentes intensas de luz, de forma que las medidas sean más estables.
