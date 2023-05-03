# Medidor AireCiudadano: sensor de PM2.5, opcional: humedad y temperatura, VOCs y NOx

![Logo](https://github.com/danielbernalb/aireciudadano/blob/main/img/Logo_aireciudadano2.jpg)

Página web: https://aireciudadano.com/

Página de la plataforma de medidores: https://sensor.aireciudadano.com/

Guía de construcción medidor AireCiudadano versión fácil
========================================================

En esta guía construiremos una versión sencilla de un medidor de Material Particulado AireCiudadano con materiales que se consiguen fácilmente en el mercado. Se usarán los 3 sensores de Material Particulado compatibles en AireCiudadano: Sensirion SPS30, Sensirion SEN5X y Plantower PMS7003.

![](https://aireciudadano.com/wp-content/uploads/IMG_20230126_154103-1024x576.jpg)

**1\. Materiales:**
-------------------

El medidor tiene las mismas partes, lo que varía es el modelo de sensor de Material Particulado.

![](https://aireciudadano.com/wp-content/uploads/IMG_20230126_153505-001-1024x577.jpg)

### **1\. Board Wemos D1 o ESPduino 32.**

[Board muy popular y económica](https://www.google.com/search?q=wemos+d1+board), se consigue en cualquier país, en Latam por Mercadolibre o tiendas de electrónica.

### **2\. Sensor, opciones:**

#### **2.1. Sensirion SPS30**

<img src="https://aireciudadano.com/wp-content/uploads/IMG_20230126_153505-002-1024x766.jpg"  width="35%" height="35%">

[El SPS30](https://sensirion.com/products/catalog/SPS30/) es el que más usamos en AireCiudadano por su calidad aunque es el más costoso del mercado. Tiene certificación de calidad MCERTS.

Link compra [Mouser](https://co.mouser.com/ProductDetail/Sensirion/SPS30?qs=lc2O%252BfHJPVbEPY0RBeZmPA%3D%3D), [Digikey](https://www.digikey.com/es/products/detail/sensirion-ag/SPS30/9598990).

**A favor:** excelente desempeño (rapidez, repetibilidad y estabilidad), tamaño.

**En contra:** el más costoso, importación desde USA.

#### 2.**2\. Sensirion SEN5X**

<img src="https://aireciudadano.com/wp-content/uploads/IMG_20230126_153332-001-1024x768.jpg"  width="35%" height="35%">

Una opción más económica que el SPS30 y también de buena calidad. Hay 3 modelos: [SEN55](https://sensirion.com/products/catalog/SEN55/), [SEN54](https://sensirion.com/products/catalog/SEN54/) y [SEN50](https://sensirion.com/products/catalog/SEN50/).

Link compra [Mouser](https://co.mouser.com/ProductDetail/Sensirion/SEN55-SDN-T?qs=MyNHzdoqoQLyWUzejWkZfg%3D%3D), [Digikey](https://www.digikey.com/es/products/detail/sensirion-ag/SEN55-SDN-T/16342756).

**A favor:** buen desempeño, buena marca y precio intermedio.

**En contra:** tamaño grande, importación desde USA.

**2.3. Plantower PMS7003.**

<img src="https://aireciudadano.com/wp-content/uploads/IMG_20230126_153852-001-1024x768.jpg"  width="35%" height="35%">

[El PMS7003](https://www.espruino.com/datasheets/PMS7003.pdf) es el más barato y popular, tiene un aceptable desempeño.

**A favor:** desempeño aceptable, bajo costo, se consigue en Colombia, China, USA y en muchos países.

**En contra:** desempeño inferior VS Sensirion, conexión más difícil. Lento, fiabilidad media, regular repetitividad.

### **3\. Cable jumper macho macho de 10cm.**

[Cable muy popular y fácil de consegui](https://www.google.com/search?q=cable+jumper+macho+macho)r en Latinoamérica: Mercadolibre o tiendas de electrónica.

**2\. Conexiones para cada sensor:**
------------------------------------

### **Sensirion SPS30**

![](https://aireciudadano.com/wp-content/uploads/Con4_1.jpg)

### **Sensirion S**EN5X

![](https://aireciudadano.com/wp-content/uploads/Con3_1.jpg)

### **Plantower PMS7003**

![](https://aireciudadano.com/wp-content/uploads/Con5_1.jpg)

Video explicativo paso a paso:

[![](https://aireciudadano.com/wp-content/uploads/VideoCP.jpg)](https://www.youtube.com/watch?v=Jkhu_hSVWPI "Video ensamblaje")


**3\. Programar el Firmware (código en la board)**
--------------------------------------------------

La opción más sencilla y práctica es el instalador web AireCiudadano:

[https://aireciudadano.com/programador-de-medidores-aireciudadano/](https://aireciudadano.com/programador-de-medidores-aireciudadano/)

Selecciona el uso que le quieres dar a de tu medidor y versión sin pantalla:

1\. Si lo que necesitas es hacer mediciones fijas, ósea desde el mismo lugar, transmitiendo a tu wifi escoge las opciones “Fijo – Wifi” dependiendo de si tienes pantalla o no.

![](https://aireciudadano.com/wp-content/uploads/Sin-titulo-3.jpg)

2\. Si lo que necesitas es hacer mediciones móviles usando batería escoge las opciones “Móvil – Bluetooth”:

<img src="https://aireciudadano.com/wp-content/uploads/2022/06/IMG_20220609_122753-975x1024.jpg"  width="50%" height="50%">

En este video se explica detalladamente cómo programar el firmware, hasta el minuto 3:30 aplica para versión móvil y fija, después sólo a la versión fija:

[![](https://aireciudadano.com/wp-content/uploads/VideoIns.jpg)](https://youtu.be/IA3pZgo-LZg "Video instalación")


**4\. Modos de uso del medidor: Wifi y Bluetooth.**
---------------------------------------------------

### **4.1. Modo fijo Wifi:**

Puedes conectar tu medidor AireCiudadano a una red Wifi y dejarlo en modo fijo – estático (en el mismo lugar todo el tiempo). Así reportaras a la red fija AireCiudadano y podrás ver los datos monitoreados en la página web:

[https://sensor.aireciudadano.com/](https://sensor.aireciudadano.com/)
![](https://aireciudadano.com/wp-content/uploads/MapaAC-1024x463.jpg)


Para la configuración del medidor en Modo fijo sigue el siguiente video:

[![](https://aireciudadano.com/wp-content/uploads/VideoWifi.jpg)](https://youtu.be/-J3zXF87OIE "Video configuraciíon Wifi")

### **4.2. Modo móvil Bluetooh:**

Debes colocar una batería externa de teléfono móvil para realizar mediciones móviles. Simplemente al encender el medidor comienza a grabar los datos según el tiempo programado que le tengas: 2, 10, 60, 120, 300, 600, 3600 o 10800 segundos (10 segundos por defecto).

Baja la App **Sensirion MyAmbience** para conectarte al medidor AireCiudadano y monitorear, dibujar, guardar y exportar tus datos:

*   [Download for Android](https://play.google.com/store/apps/details?id=com.sensirion.myam)

*   [Download for iOS](https://apps.apple.com/ch/app/sensirion-myambience-co2/id1529131572)

<img src="https://aireciudadano.com/wp-content/uploads/photo_2023-01-27_22-07-30-511x1024.jpg"  width="30%" height="30%">

En este pdf del fabricante página 5, encontrarás una buena explicación del funcionamiento de la App:

[https://sensirion.com/media/documents/F8D0AE28/617BC2FB/Sensirion\_Humidity\_Sensors\_SHT4x\_Smart-Gadget\_User-Guide.pdf](https://sensirion.com/media/documents/F8D0AE28/617BC2FB/Sensirion_Humidity_Sensors_SHT4x_Smart-Gadget_User-Guide.pdf)

Cualquier duda que tengas puedes contactarnos por correo electrónico ([info@aireciudadano.com](mailto:info@aireciudadano.com)) o a nuestras redes sociales [@aireciudadano](https://twitter.com/AireCiudadano)[  
](https://sensirion.com/media/documents/F8D0AE28/617BC2FB/Sensirion_Humidity_Sensors_SHT4x_Smart-Gadget_User-Guide.pdf)