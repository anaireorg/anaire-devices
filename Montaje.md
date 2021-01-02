# En elaboración actualmente, información inestable en proceso de actualización
----

# 3D Box design
 - https://www.thingiverse.com/thing:4678398/files

 ![Cajas impresora 3D](https://github.com/anaireorg/anaire-devices/blob/main/images/WhatsApp%20Image%202020-12-07%20at%2011.47.58.jpeg)

# Montaje

1. Soldar los cuatro pines en el sensor de CO2 donde está indicado en la imagen, para poder pincharlo en la breadboard

![Pines del sensor de CO2 MHZ-14A](https://github.com/anaireorg/anaire-devices/blob/main/images/Pines_CO2_ANAIRE.png)

2. Disponer y cablear los elementos como se muestra en las siguientes imágenes.
- Comprobar que los dos jumper en la fuente de alimentación están en la posición de 5Vcc
- Se recomienda poner unas gotitas de pegamento en la base de la fuente de alimentación, para que no se pueda mover de su sitio:

Componentes:
![Montaje](https://github.com/anaireorg/anaire-devices/blob/main/images/medida_photo_2020-12-07_22-28-34.png)

Conexiones:
![Conexiones](https://github.com/anaireorg/anaire-devices/blob/main/images/Connections_NodeMCU_LUA_Amica_V2_Schematic.png)

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

  ![Web server](https://github.com/anaireorg/anaire-devices/blob/main/images/Captura_calibration.PNG)

  - Hacer click en "here" en el enlace mostrado en la primera línea, y esperar a que se termine la cuenta atrás en el display del dispositivo y se vuelvan a mostrar medidas. El proceso se completa en aproximadamente 20 minutos.

![Calibrating](https://github.com/anaireorg/anaire-devices/blob/main/images/calibrating_photo_2020-12-07_22-01-00.jpg)

**Una vez calibrado el dispositivo está listo para su uso. Aún en caso de errores de conexión WiFi o MQTT el dispositivo seguirá mostrando las medidas actualizadas de CO2 en el display. El proceso de calibración debe repetirse una vez cada 30 días.**
