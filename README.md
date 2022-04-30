# Medidor AireCiudadano: sensores de PM2.5, humedad y temperatura. Opcional VOC y NOx

Página web: https://aireciudadano.com/

Página de la plataforma de medidores: https://sensor.aireciudadano.com/

///////////////////////////////////////////////
AireCiudadano medidor Fijo - Medidor de PM2.5 abierto, medición opcional de humedad y temperatura.
Más información en: aireciudadano.com
Este firmware es un fork del proyecto Anaire (https://www.anaire.org/) recomendado para la medición de CO2.
2022 info@aireciudadano.com
///////////////////////////////////////////////

Pendientes:
OK: Nombre de la estacion con modelo de sensor, board  y etiqueta propia si es posible
OK: Programacion de modelo de sensor por portal cautivo
BT funcionando en este codigo y sin WIFI y encendiendo sensor con pin enable
OK: TTGO T Display funcionando
Agregar comparacion de valores de PM25 para emoticons y colores
OLED funcionando
OK: AireCiudadano Splash Screen
OK: Valor de RSSI para modo wifi
OK: Añadir coordenadas GPS
OK: Añadir VOCs y NOx para SEN5X
OK: Revisar presicion envio de float, si cortarla o dejarlo al usuario
OK: Variable Sensor de exteriores o interiores, ExternalSensor via mqtt Var1 datavar1
OK: Tiempo de muestreo
OK: Revisar diferencia entre Sensor characteristics y Numero ID de la configuracion del sensor
OK: Revisar Update por Portal Cautivo
Guardar configuracion de sensores, board, display y demás en el Portal Cautivo

MODIFICACIONES EXTERNAS:
Modificado WifiManager.cpp para que cuando ingrese al Config del portal cautivo pase a 180 segundos y no 10:
_configPortalTimeout = 300000;   // New Config Portal Timeout
DEBUG_WM(DEBUG_VERBOSE,F("New Config Portal Timeout: 300 seconds"));
// Modificado WiFiManager.template.htlm y template.h en texto: Configure WiFi por Configure WiFi and Sensor
