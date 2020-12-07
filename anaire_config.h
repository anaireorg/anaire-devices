// Device data available on http://demo.anaire.org:30300/antonio01 user "demo" password "demo"

// DEVICE
char anaire_device_id[] = "antonio01";      // Unique device id
int CO2ppm_warning_threshold = 700;                   // Warning threshold
int CO2ppm_alarm_threshold = 900;                     // Alarm threshold

// LOCAL CONFIGURATION: WiFi network configuration
char ssid[] = "anaire";                  // your network SSID (name)
char password[] = "mel0nc0njam0n";       // your network password (use for WPA, or use as key for WEP)

// CLOUD CONFIGURATION: remote app url
char cloud_server_address[] = "demo.anaire.org";     // server public IP address
int cloud_app_port = 30183;                          // cloud application port
