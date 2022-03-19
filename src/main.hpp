void Display_Init();
void Display_Splash_Screen();
void Get_Anaire_DeviceId();
void Read_EEPROM();
void Print_Config();
void Button_Init();
void Connect_WiFi();
void Init_MQTT();
void Setup_Sensor();
void Update_Display();
void Read_Sensor();
void Evaluate_CO2_Value();
void Update_Display();
void Send_Message_Cloud_App_MQTT();
void MQTT_Reconnect();
void Init_MQTT();
void Check_WiFi_Server();
void Print_WiFi_Status();
void Do_Calibrate_Sensor();
void Set_AutoSelfCalibration();
void Write_EEPROM();
void Start_Captive_Portal();
void Suspend_Device();
void Write_EEPROM();
void MQTT_Reconnect();
void Evaluate_CO2_Value();
void Evaluate_CO2_Value();
void Set_Temperature_Offset();
void Set_Altitude_Compensation();
void Do_Calibrate_Sensor();
void Set_AutoSelfCalibration();
void Suspend_Device();
void Wipe_EEPROM();
void Firmware_Update();
void Set_Measurement_Interval();

void displayBatteryLevel(int colour);
void displayWifi(int colour_1, int colour_2, boolean active);
void displayBuzzer(int colour, boolean active);
void Receive_Message_Cloud_App_MQTT(char* topic, byte* payload, unsigned int length);
void (WiFiEvent_t event);

#define TFT_SLPIN   0x10
#define TFT_DISPOFF 0x28