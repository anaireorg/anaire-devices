# Medidor AireCiudadano: sensores de PM2.5, humedad y temperatura. Opcional VOC y NOx

![Logo](https://github.com/danielbernalb/aireciudadano/blob/main/img/Logo_aireciudadano2.jpg)

Página web: https://aireciudadano.com/

Página de la plataforma de medidores: https://sensor.aireciudadano.com/

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

//
// Version ESP8266:

//          Revisar porque muchas veces no carga bien el portal cautivo, queda en blanco la pagina. Parece ser si hay mas de 20 router por mostrar paila:

//          Aqui revisar: https://github.com/tzapu/WiFiManager/blob/master/WiFiManager.cpp
//
// Cambio en el archivo WifiEspClient.cpp:

// size_t WiFiEspClient::print(const FlashStringHelper ifsh)

//{

//	return printFSH(ifsh, false);

//}

//

// MODIFICACIONES EXTERNAS:

// Modificado WifiManager.cpp para que cuando ingrese al Config del portal cautivo pase a 180 segundos y no 10:

// configPortalTimeout = 300000;   // New Config Portal Timeout

//  DEBUG_WM(DEBUG_VERBOSE,F("New Config Portal Timeout: 300 seconds"));

// Modificado template.h en texto: Configure WiFi por Configure WiFi and Sensor & Show Password
