# Anaire PiCO2

*@2021 Anaire www.anaire.org*

---

<p align="center">
  <img src="docs\PiCO2 click.jpg" width="60%" height="60%" />
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

El medidor Anaire PiCO2 es un dispositivo basado en un microcontrolador ESP32 en una tarjeta de control [LILYGO® TTGO T-Display ESP32 WiFi And Bluetooth](https://github.com/Xinyuan-LilyGO/TTGO-T-Display), que proporciona también conectividad WiFi y Bluetooth, y permite su programación desde el IDE de Arduino. La tarjeta tiene integrado un display en color para mostrar las mediciones e indicaciones de estado. Adicionalmente el dispositivo equipa un zumbador para poder emitir alertas sonoras y una batería recargable para facilitar la movilidad.

Para realizar las medidas se conecta un sensor de CO2 de tecnología NDIR, que adicionalmente realiza medidas de temperatura y humedad:  [*Sensirion SCD30*](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-scd30/). El Sensirion SCD30 es un sensor de alta calidad y gran velocidad de reacción: tiene mayor precisión que la mayoría de los sensores de su mismo tipo, 30 ppm frente a los 50 ppm del Winsen MH-Z14A; incorpora sensor de temperatura y humedad, con lo que no hay que añadir un sensor adicional para medir temperatura y humedad, facilitando el montaje del medidor. Además de permitir mecanismos de compensación por altitud y temperatura; tiene un mecanismo de autocalibración más robusto, que hace uso de las mediciones de los últimos 30 días, en lugar de las últimas 24 horas, como en el caso del MH-Z14A. Su respuesta es también más rápida ante cambios atmosféricos (por ejemplo, al introducir ventilación en un espacio cerrado). Los inconvenientes del Sensirion SCD30 son que tiene un precio más elevado, que es más frágil durante su manipulación y que tiene menos opciones de compra y, normalmente, mayores plazos de entrega. Para comprar el Sensirion SCD30 hay que recurrir normalmente a plataformas especializadas en componentes electrónicos.

La alimentación del dispositivo se realiza directamente a través del puerto USB-C de la tarjeta de control [LILYGO® TTGO T-Display ESP32 WiFi And Bluetooth](https://github.com/Xinyuan-LilyGO/TTGO-T-Display), el mismo que se utiliza para programarla y para comunicarse con ella. La batería incorporada se carga automáticamente cuando se conecta un cable USB-C alimentado. Se recomienda utilizar fuentes de alimentación (enchufes USB, puertos USB en ordenadores, etc.) que puedan proporcionar al menos 1.000 mA.   

A continuación, se enumeran los elementos citados:

* Tarjeta de control basada en microcontrolador ESP32 [LILYGO® TTGO T-Display ESP32 WiFi And Bluetooth](https://github.com/Xinyuan-LilyGO/TTGO-T-Display)  

<p align="center">
  <img src="docs\TTGO.png" width="35%" height="35%" />  
</p>

* Sensor de CO2, temperatura y humedad [*Sensirion SCD30*](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-scd30/). Para la conexión del sensor de CO2 es preciso conectar 4 pines y soldarlos en las posiciones indicadas en el proceso de fabricación. Se pueden comprar pines como [los que aparecen en este enlace](https://www.amazon.es/Pin-Header-Way-Straight-Pitch/dp/B00QXBRCKG/ref=sr_1_5?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=pin+header&qid=1609617099&s=electronics&sr=1-5), por ejemplo, y cortar trozos de 4 pines con un alicate de corte o unas tijeras.

<p align="center">
  <img src="docs\SCD30 transparente.png" width="40%" height="40%" />
</p>  

* Caja -Anaire Box- para impresora 3D. Los ficheros para la impresión se pueden encontrar en [este enlace](https://www.thingiverse.com/thing:4854504).

<p align="center">
  <img src="docs\Caja 3D 1.png" width="60%" height="60%" />
</p>    
<p align="center">
  <img src="docs\Caja 3D 2.png" width="60%" height="60%" />
</p>
<p align="center">
  <img src="docs\Caja 3D 3.png" width="40%" height="40%" />
</p>  

* Zumbador [- AZDelivery Active Buzzer](https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2)  

<p align="center">
  <img src="docs\Zumbador.png" width="30%" height="30%" />  
</p>

* [Batería Recargable de Litio con conector Micro JST 1.25 3.7V y al menos 1.000 mAh de capacidad](https://www.amazon.es/dp/B087LTZW61/ref=cm_sw_r_cp_awdb_imm_BPVY2QA8X3P5GB9NQ55C)

<p align="center">
  <img src="docs\Batería Recargable de Litio Micro JST 1.25 3.7V.png" width="40%" height="40%" />
</p>

* [Cables Dupont hembra-hembra de 10cm de longitud](https://es.aliexpress.com/item/32223527499.html?acnt=439-079-4345&aff_platform=aaf&ds_e_device=c&albcp=13172094689&ds_e_product_id=es32223527499&ds_url_v=2&ds_dest_url=https%3A%2F%2Fs.click.aliexpress.com%2Fdeep_link.htm%3Faff_short_key%3DUneMJZVf&ds_e_product_group_id=296904914040&pvid=eb11631d-971c-4829-b794-e9f23d8cb417&sk=UneMJZVf&scm=1007.23534.123999.0&ds_e_adid=522675293728&terminal_id=44af2aac40424e0b80b3a282ef6f308e&needSmbHouyi=false&albbt=Google_7_shopping&ds_e_product_channel=online&src=google&ds_e_product_country=ES&aff_fcid=cfebeae5e65a4635b7cc3e2f3b15027f-1621885142306-06674-UneMJZVf&gclid=Cj0KCQjwna2FBhDPARIsACAEc_X87Ff7qCzv9SHcW1JGMfc1Hfr8fuY_dryH4gYq7SQr2iEJrtdeEuwaAoGpEALw_wcB&albag=123014812712&aff_fsk=UneMJZVf&albch=shopping&ds_e_network=u&albagn=888888&ds_e_product_language=es&isSmbAutoCall=false&ds_e_product_merchant_id=107407105&aff_trace_key=cfebeae5e65a4635b7cc3e2f3b15027f-1621885142306-06674-UneMJZVf&rmsg=do_not_replacement&gclsrc=aw.ds)

<p align="center">
  <img src="docs\Cables Dupont.png" width="80%" height="80%" />
</p>

* Será necesario un cable válido para conectar desde un puerto USB del ordenador al puerto Micro USB-C de la tarjeta TTGO T-Display, para poder cargar el software del dispositivo. Este mismo cable puede ser utilizado para la alimentación eléctrica desde un ordenador, o desde un enchufe USB o incluso desde una batería portátil USB. También podría utilizarse, una vez cargado el software, cualquier otro tipo de dispositivo que tenga un conector USB-C y proporcione al menos 1.000 miliamperios. Por ejemplo, [este cable](https://www.amazon.es/dp/B01N4OKUB8/ref=sspa_dk_detail_0?psc=1&pd_rd_i=B01N4OKUB8&pd_rd_w=dwqWt&pf_rd_p=5f25be7b-80cd-4c25-8d91-1b1227f5fe0a&pd_rd_wg=YOc8a&pf_rd_r=65KA0NS0Q3TFHFFG71BT&pd_rd_r=1ea810dc-c0cf-4076-9c01-6a1168e0c782&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExWVdXWkI3STRMR0JPJmVuY3J5cHRlZElkPUEwNzg0ODEwMlRGUUNCTFU5VzREVSZlbmNyeXB0ZWRBZElkPUEwNzY1NTk5NkUyNFdHOUZWOTNRJndpZGdldE5hbWU9c3BfZGV0YWlsX3RoZW1hdGljJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==), o [este alimentador](https://www.amazon.es/UGREEN-Cargador-Delivery-Ultrarrapido-iPhone/dp/B07QRQTKCZ/ref=sr_1_4?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=cargador+usb-c&qid=1621421985&s=electronics&sr=1-4)

<a name="id2"></a>

# 2. Software

El software se puede cargar en el dispositivo de dos formas: 

- Utilizando el [entorno de desarrollo (IDE) de Arduino](https://www.arduino.cc/en/software), exactamente igual que una tarjeta arduino. Hay que instalar algunos componentes de software adicionales en el entorno de Arduino, principalmente diversas librerías. La descripción completa de la instalación del entorno de *software* se puede encontrar al inicio del archivo de código fuente del dispositivo medidor [anaire.PiCO2.ino](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.PiCO2/anaire.PiCO2/anaire.PiCO2.ino).

- Mediante un mecanismo sencillo basado en la carga del binario ya compilado en la tarjeta TTGO T-Display, como se describe más adelante, en el apartado de fabricación.

En cualquier caso, la instalación del entorno completo de desarrollo no será necesaria si no se requiere hacer modificaciones en el software. 

Para cualquiera de las dos opciones será necesario instalar previamente en un ordenador con sistema operativo Windows10 el [driver USB-UART](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (conversión USB a puerto serie) para poder comunicarse con la tarjeta TTGO T-Display.

<a name="id3"></a>

# 3. Fabricación

El montaje que se propone pretende simplificar al máximo el proceso de fabricación, manteniendo las máximas prestaciones técnicas. Los componentes se encajan en la caja de impresión 3D, y las conexiones se realizan mediante cables Dupont hembra-hembra, minimizando así las necesidades de soldadura. Sólo será necesario soldar 4 pines en el sensor SCD30 y 7 pines en la tarjeta TTGO T-Display (en el caso de que no se haya comprado con los pines ya soldados).

## Esquema de conexiones:  
<p align="center">
  <img src="docs\PiCO2 Conexiones.png" width="60%" height="60%" />
</p>  

## Detalle del sensor SCD30  
Hay que soldar 4 pines como se muestra en la siguiente imagen  
<p align="center">
  <img src="docs\pines_soldados_SCD30.jpg" width="60%" height="60%" />
</p>  

## Montaje final  

Recomendamos que primero se conecten los componentes, se cargue el *software*, siguiendo las instrucciones del apartado siguiente, y se verifique así el funcionamiento del medidor.


<p align="center">
  <img src="docs\Fabricación 2.jpg" width="60%" height="60%" />
</p>  
<p align="center">
  <img src="docs\Fabricación 1.jpg" width="60%" height="60%" />
</p>  

A continuación, se colocan los componentes mecánicamente en su posición dentro de la caja, y se vuelve a verificar el funcionamiento correcto.

Para ello, primero se coloca la pieza cuadrada pequeña alrededor de la pantalla, como se muestra en la imagen:

<p align="center">
  <img src="docs\Fabricación 3.jpg" width="60%" height="60%" />
</p>

Luego, se inserta la placa ajustando el conector para la alimentación y se encaja en la pestaña que hay en el lado opuesto. Se ajusta el zumbador en el hueco circular del lateral y, finalmente, se ajusta el sensor en las pestañas que hay debajo de donde va colocada la placa.

<p align="center">
  <img src="docs\Fabricación 4.jpg" width="60%" height="60%" />
</p>

Finalmente se debe proceder a la conexión a la red WiFi y la calibración de referencia del sensor. Se puede activar el mecanismo de autocalibración del SCD30, pero para que funcione correctamente hay que tenerlo encendido durante una semana garantizando su exposición al aire libre (se trata de medir un valor mínimo de CO2) durante al menos una hora al día.

El siguiente vídeo muestra de forma detallada el proceso montaje:

[![Montaje Anaire PiCO2](http://img.youtube.com/vi/GUXPi4h12Kw/0.jpg)](https://youtu.be/GUXPi4h12Kw)

## Carga del software

Para la carga del software hay dos opciones: carga del binario compilado, o carga desde el código fuente. Como ya se ha explicado anteriormente, si no se desea modificar el software se recomienda la primera opción, ya que es mucho más sencilla y rápida.

En cualquiera de los dos casos es preciso instalar previamente en un ordenador con sistema operativo Windows 10 el [driver USB-UART](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (conversión USB a puerto serie) para poder comunicarse con la tarjeta TTGO T-Display.

### Carga del binario compilado
Estas instrucciones son únicamente válidas para ordenadores con el sistema operativo Windows 10.

Asegure que se tiene en un directorio los siguientes archivos, disponibles en la carpeta [anaire.PiCO2](https://github.com/anaireorg/anaire-devices/tree/main/Anaire.PiCO2/anaire.PiCO2):
  * anaire.PiCO2.ino.esp32.bin
  * boot_app0.bin
  * bootloader_qio_80m.bin
  * esptool.exe
  * flash_device.bat

Ejecute los siguientes pasos en esa carpeta:
1. Conecte el medidor PiCO2 al ordenador mediante un cable USB-C. En el administrador de dispositivos de windows podrá identificar el puerto COM asignado por el sistema operativo al medidor PiCO2.

<p align="center">
  <img src="docs\Identificación de puerto COM.png" width="80%" height="80%" />
</p>

2. Haga click derecho en el archivo [flash_device.bat](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.PiCO2/anaire.PiCO2/flash_device.bat) y seleccione la opción de *editar*

3. Donde pone *--port COMXX* sustituya el puerto COMXX para poner el que haya identificado previamente en el paso 1, seleccione *archivo->guardar* y cierre la ventana

<p align="center">
  <img src="docs\Edición de puerto COM.png" width="100%" height="100%" />
</p>

4. Haga doble click en el archivo [flash_device.bat](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.PiCO2/anaire.PiCO2/flash_device.bat). Al acabar la ejecución pedirá que se pulse cualquier tecla para cerrar la ventana. El medidor PiCO2 arrancará con la última versión de *software* disponible en el archivo [anaire.PiCO2.ino.esp32.bin](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.PiCO2/anaire.PiCO2/anaire.PiCO2.ino.esp32.bin)

<p align="center">
  <img src="docs\Ejecución carga de binario.png" width="100%" height="100%" />
</p>

### Carga desde el código fuente

1. Instale el [entorno de desarrollo de Arduino](https://www.arduino.cc/en/software).

2. Instale los componentes de software requeridos en el entorno de Arduino, tal como se describe al inicio del archivo de código fuente del dispositivo medidor [anaire.PiCO2.ino](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.PiCO2/anaire.PiCO2/anaire.PiCO2.ino).
Para ello realice los siguientes pasos:

3. Cargue el programa [anaire.PiCO2.ino](https://github.com/anaireorg/anaire-devices/blob/main/Anaire.PiCO2/anaire.PiCO2/anaire.PiCO2.ino) en el IDE de Arduino utilizando la opción *Archivo->Abrir*

4. Conecte mediante un cable USB-C el PiCO2 al ordenador

5. Seleccione el puerto apropiado en *Herramientas -> Puerto*. Si sólo tiene un PiCO2 conectado, sólo debería aparecer una opción para seleccionar. Si no aparece ninguna, o bien no se ha instalado correctamente el [driver USB-UART](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers), o bien el cable USB-C o la tarjeta TTGO T-Display son defectuosos.

6. Seleccione el esquema de particionado de la tarjeta tal como se muestra en la imagen a continuación:

<p align="center">
  <img src="docs\Partition Scheme.png" width="100%" height="100%" />
</p>

7. Presione el botón de carga, que se indica en el IDE con una flecha apuntando hacia la derecha, y espere a que se compile y se descargue el software en la tarjeta TTGO T-Display.

<p align="center">
  <img src="docs\Descarga de SW desde el IDE de Arduino.png" width="100%" height="100%" />
</p>

El proceso puede durar varios minutos, dependiendo de la potencia de cálculo del ordenador. Al término del proceso en el IDE de Arduino se mostrará un mensaje indicando que se ha escrito el 100% del programa en la tarjeta, y el dispositivo PiCO2 se reiniciará mostrando durante varios segundos la imagen de Anaire en la pantalla, para a continuación comenzar a mostrar medidas cada 10 segundos.

<a name="id4"></a>

# 4. Configuración

Para completar la configuración del dispositivo PiCO2 hay que conectarlo a una red WiFi y realizar la calibración de referencia.

## Configuración de la conexión WiFi
   * Pulse 3 veces consecutivas el botón superior para arrancar el portal cautivo, es decir, una red WiFi y un servidor web que le permiten modificar la configuración del PiCO2.
   * Se creará una red WiFi abierta con el nombre AnaireWiFi_XXXXXX, donde XXXXXX es el identificador del dispositivo. La pantalla mostrará sobre fondo naranja el nombre de la red WiFi creada.
   * Conecte a esa red con un PC, tablet o teléfono móvil
   * Una vez conectado a la red introduzca la dirección IP **192.168.4.1** en su navegador. De esta forma se accede al portal de configuración del dispositivo
   * En el portal puede escoger una red WiFi entre las detectadas, e introducir la contraseña de la red. Pulse en el botón *Save* tras escribir la contraseña.
     * Si todo ha ido correctamente el dispositivo se reiniciará con conexión a la nueva red WiFi y podrá ver el icono de la conexión WiFi en la esquina superior izquierda de la pantalla.
     * En caso de utilizar encriptación WPA2-Enterprise deberá espera a que el PiCO2 se reinicie automáticamente al cabo de 2 minutos.
     * Si ha habido algún problema de conexión (por ejemplo, si se ha introducido una contraseña errónea), el dispositivo mostrará el icono de la red WiFi en la esquina superior izquierda del display tachado, y deberá repetir el procedimiento.
   * En el portal se puedan modificar otros parámetros de configuración, como un usuario y contraseña para redes WPA2-Enterprise, o la conexión a la aplicación en la nube (MQTT server y puerto).

## Calibración de referencia
Sitúe el medidor PiCO2 al aire libre, protegido del sol, de otras fuentes de luz intensas y de las corrientes de aire (por ejemplo, dentro de una caja de cartón en el alféizar de la ventana), y arranque el proceso de calibración mediante alguno de los siguientes métodos:
- Presione el botón inferior durante más de 1 segundo
- Identifique la dirección IP del dispositivo pulsando brevemente el botón superior, e introduzca la dirección en un navegador de un dispositivo conectado a la misma red WiFi. En el servidor web haga click en el enlace que permite iniciar la calibración de referencia del dispositivo.
- Acceda a la aplicación en la nube, entrando en la opción de editor de parámetros. Para ello debe tener permisos de administrador en la aplicación. Active la opción denominada *Forced calibration* y haga click en el botón *Update*. de esta forma podrá iniciar la calibración a través de Internet.

En cualquiera de los métodos anteriores la pantalla del PiCO2 se pondrá de color naranja e iniciará una cuenta atrás de 60 segundos para indicar el tiempo restante hasta que se complete la calibración forzada. No interfiera con el dispositivo, ni lo manipule o lo acerque a fuentes de CO2 (como la respiración) mientras dure la cuenta atrás.

Al término del proceso el dispositivo se reiniciará y volverá a tomar medidas, tomando como referencia el valor de 420 ppm (este valor se puede modificar desde la aplicación en la nube). El proceso no puede ser interrumpido, deberá esperar a su término al cabo de 60 segundos en cualquier caso.

<a name="id5"></a>

# 5. Uso

Para empezar a realizar medidas sólo tiene que alimentar el PiCO2 con un cable USB-C. Cuando la batería ya esté parcialmente cargada podrá incluso prescindir de la alimentación durante varias horas y llevarse el PiCO2 para realizar medidas itinerantes. Un icono con forma de pila en la parte inferior izquierda de la pantalla le mostrará la indicación del nivel de carga disponible.

En caso de que el valor medido de CO2 supere el umbral de aviso, el dispositivo empezará a emitir un pitido intermitente, y el fondo de la pantalla se mostrará de color amarillo. Si supera el valor del umbral de alarma, el pitido intermitente será más largo, y el fondo de la pantalla se mostrará de color rojo.

Los valores por defecto de los umbrales son 700 PPM para el aviso y 1.000 PPM para la alarma. Estos valores se pueden modificar desde la aplicación de Anaire en la nube.

Si se desea detener la alarma acústica presione durante más de un segundo el botón superior. Vuelva a hacer lo mismo cuando desee activarla de nuevo. En el display se mostrará el estado de la alarma mediante un icono que muestra un altavoz (activa), o un altavoz tachado (desactivada).

El PiCO2 tiene un identificador único (ID) generado a partir de la dirección MAC de la interfaz WiFi de cada tarjeta TTGO T-Display. Este ID es un texto hexadecimal de 6 caracteres, utilizado para el acceso a las medidas en Internet y la integración del medidor en la aplicación en la nube de Anaire. No es preciso realizar ninguna configuración sobre este identificador, se genera de forma automática para cada tarjeta TTGO. Este ID se utiliza para identificar las medidas del dispositivo en la aplicación en la nube, así como para identificar la red WiFi que genera el propio PiCO2 cuando se arranca el modo de configuración.

## Botones
El PiCO2 tiene dos botones a la derecha de la pantalla, uno superior y otro inferior. Puede usar los botones para realizar las siguientes funciones:

### Botón superior
- Pulsación corta - Muestra información del dispositivo: ID, versión de *software*, red WiFi, dirección IP, dirección MAC, potencia WiFi (RSSI), estado de la alarma acústica (SÍ/NO), estado de la calibración (forzada/auto)
- Pulsación larga (durante más de 1 segundo) - Conmuta el estado de la alarma acústica
- Doble pulsación - Duerme el dispositivo para ahorrar batería cuando no sea necesario utilizarlo; para despertarlo, vuelva a pulsar el botón superior
- Triple pulsación - Arranca el portal cautivo que permite configurar la conexión WiFi y la dirección del servidor MQTT de la aplicación Anaire en la nube a la que enviar las medidas

### Botón inferior
- Pulsación corta - Muestra información del uso de los botones
- Pulsación larga (más de 1 segundo) - Inicia el proceso de calibración forzada. Este proceso dura 60 segundos y no se puede interrumpir.
- Doble pulsación -  Reinicia el PiCO2
- Triple pulsación - Activa el mecanismo de autocalibración del PiCO2, dejando sin efecto la calibración de referencia que se haya podido realizar anteriormente.

## Servidor Web

Si se conecta con un dispositivo a la misma red WiFi a la que esta conectado el medidor PiCO2, accediendo a la dirección IP del medidor mediante un navegador se puede acceder al servidor web del PiCO2, pudiendo así acceder a información detallada del dispositivo, a las últimas medidas, e incluso pudiendo realizar diversas acciones de configuración, como se puede ver en la siguiente imagen:

<p align="center">
  <img src="docs\Servidor Web.png" width="80%" height="80%" />
</p>

## Acceso por internet a las medidas
Si se ha conectado el PiCO2 a una red WiFi con acceso a internet y se han dejado los valores por defecto de conexión a la aplicación en la nube, se puede acceder a los valores medidos por el dispositivo en los siguientes enlaces. Sustituya *ID_del_dispositivo* por el ID del PiCO2 en cuestión para acceder a las medidas:

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

<a name="6"></a>

# 6. Diagnóstico

Conecte el dispositivo al PC utilizando un cable USB-C. Arranque el monitor serie del IDE de Arduino con la opción *Herramientas -> Monitor serie*. Se abrirá una nueva ventana en la que se imprimirán todos los mensajes emitidos por el dispositivo durante su funcionamiento, que ayudarán a diagnosticar los posibles problemas. Se recomienda reiniciar el dispositivo pulsando dos veces el botón inferior para forzar el reinicio y poder observar un ciclo completo de funcionamiento desde el inicio del dispositivo.

Inicialmente el dispositivo mostrará la medida de 0 PPM. Si esta medición no cambia antes de un minuto desde que el dispositivo esté encendido, el sensor de CO2 no está siendo detectado por la tarjeta de control TTGO T-Display. Si esto sucede se recomienda conectar el dispositivo al ordenador y observar las trazas enviadas por el medidor tal como se describe en el punto anterior, para verificar el estado del dispositivo.

En el lateral izquierdo de la pantalla del PiCO2 se muestran 3 iconos: un indicador de conexión a la red Wifi, un altavoz que indica el estado de la alarma acústica, y un indicador de la carga de la batería. Cuando no hay conexión a la red el icono correspondiente aparecerá tachado. Cuando se configure la conexión WiFi o se vuelva a tener cobertura en caso de haberla perdido, el icono dejará de estar tachado, indicando que se ha recuperado la conectividad. De forma análoga, el icono del altavoz mostrará el estado de la alarma acústica, y el icono de la pila mostrará el nivel de carga de la batería.

---

# BONUS

## Montaje sobre Breadboard (placa de prototipado)
El montaje sobre breadboard es algo que puede ser útil para proyectos educativos, o para desarrollo, o incluso en caso de no tener acceso a una impresora 3D.

Es extremadamente sencillo montar el PiCO2 sobre breadboard. El siguiente vídeo muestra el proceso en detalle para el Anaire 30ppm. Realice el mismo procedimiento, pero sustituyendo la tarjeta NodeMCU por la tarjeta TTGO T-Display, dejando de lado el display que se muestra en el vídeo (no es necesario ya que la TTGO incorpora su propia pantalla) y cargando el *software* como se ha indicado en el apartado de fabricación del PiCO2:

[![Montaje Anaire30ppm breadboard](http://img.youtube.com/vi/d87cvSwYpxk/0.jpg)](https://youtu.be/d87cvSwYpxk)

En este caso es relevante destacar que tras soldar los cuatro pines en el SCD30, utilizando éstos se puede pinchar el sensor directamente en la placa de prototipado, alineado correctamente con los pines de la tarjeta TTGP T-Display para que se verifique el cableado deseado, ahorrando así cuatro cables y facilitando el engarce mecánico del conjunto de una forma sencilla y muy efectiva.

Este montaje es muy sencillo de realizar, pero hay que tener en cuenta que la proximidad entre el sensor SCD30 y el microcontrolador en la placa TTGO T-Display puede producir que haya alteración de las medidas realizadas, especialmente las de temperatura y humedad. Adicionalmente, el sensor queda expuesto a corrientes de aire o fuentes de luz que producen alteraciones significativas de las medidas. Siempre resulta conveniente poder alojar mecánicamente el conjunto en algún tipo de caja protectora.
