///////////////////////////////////////////////////////////////////////////////////////////////////
// 20201109 Antonio López Gracia
// Get CO2, Temperature and humidity measurements and send them to the anaire cloud app
// MH-Z14A: pin 15 <-> 5Vcc Power In, pin 3 <-> GND, pin 10 <-> UART TXD, pin 18 UART RXD
// MH-Z14A CO2 sensor connected to Serial1 (pin 18-TX1, pin 19-RX1) in Arduino due
// Use programming port to get serial monitor working on Arduino IDE (the one close to power input)
///////////////////////////////////////////////////////////////////////////////////////////////////

// Command to get measurements from MH-Z14A CO2 sensor
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  

// Command to calibrate MH-Z14A CO2 sensor
byte cmdCal[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // calibrate command

// holds the received data from MH-Z14A CO2 sensor
char response[9]; 

// CO2 ppm measured value
int CO2ppm = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(9600);

  // Initialize serial port to communicate with MH-Z14A CO2 sensor
  Serial1.begin(9600);

  // Calibrate MH-Z14A CO2 sensor
  calibrate_MHZ14A();
 
  // initialize digital pin LED_BUILTIN as an output to blink, showing activity
  pinMode(LED_BUILTIN, OUTPUT);
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // clears out any garbage in the RX buffer
  while (Serial1.available())  
  {
    int garbage = Serial1.read();
  }

  // sent out read command to the sensor - 9 bytes
  Serial1.write(cmd, 9);  

  // pauses the sketch and waits for the TX buffer to send all its data to the sensor
  Serial1.flush();  

  // pauses the sketch and waits for the sensor response
  while (!Serial1.available())  
  {
    delay(0);
  }

  // once data is avilable, it reads it to a variable 9 bytes
  Serial1.readBytes(response, 9);  
  //Serial.println (response);

  // calculates CO2ppm value
  int responseHigh = (int)response[2];
  int responseLow = (int)response[3];
  CO2ppm = (256 * responseHigh) + responseLow;

  //Serial.print (responseHigh);
  //Serial.print (" <-> ");
  //Serial.println (responseLow);

  // prints calculated CO2ppm value to serial monitor
  Serial.print ("CO2ppm = ");
  Serial.println (CO2ppm);

  //delay(1000);

  // blinks LED to show activity
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(800);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);                       // wait for a second
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

// Calibrate MH-Z14A CO2 sensor
void calibrate_MHZ14A()
{
  // Write calibration command
  Serial1.write(cmdCal, 9);

  // Waits for 3 seconds
  delay(3000);
}
