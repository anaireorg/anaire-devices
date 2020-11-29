// DEVICE
char anaire_device_id[] = "antonio02";      // unique device id
int CO2ppm_alarm_threshold = 500;           // Alarm threshold, used to trigger red LED and buzzer sound alarm
int CO2ppm_warning_threshold = 410;         // Warning threshold, used to trigger yellow LED

// LOCAL CONFIGURATION: WiFi network configuration
char ssid[] = "anaire";                  // your network SSID (name)
char password[] = "mel0nc0njam0n";       // your network password (use for WPA, or use as key for WEP)

// CLOUD CONFIGURATION: remote app url
char cloud_server_address[] = "99.81.90.83";     // server public IP address
int cloud_app_port = 30183;                      // cloud application port
