#include <BluetoothSerial.h>  // Include the BluetoothSerial library
#include "HX711.h"
#include "esp_system.h"  // Include the ESP32 system header

BluetoothSerial SerialBT;  // Create an instance of the BluetoothSerial class
HX711 myScale;

// Define the HX711 data and clock pins
uint8_t dataPin  = 34;
uint8_t clockPin = 23;
uint8_t ledPin   = 5;      // Define the LED pin

void setup()
{
  pinMode(ledPin, OUTPUT); // Set the LED pin as output
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("LIBRARY VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

  uint64_t chipid = ESP.getEfuseMac(); // Get the MAC address of the ESP32
  String deviceName = "ESP32-BT-" + String((uint32_t)chipid); // Create a unique device name
  SerialBT.begin(deviceName); // Start the Bluetooth with the unique device name
  Serial.print(deviceName);
  Serial.println("\tThe device started, now you can pair it with bluetooth!");

  myScale.begin(dataPin, clockPin);
  //set mode
  myScale.set_medavg_mode();
  Serial.print(myScale.get_mode());
  
}

void loop()
{
  // Blink the LED
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);

  // Wait for a Bluetooth connection
  if (SerialBT.hasClient())
  {
    Serial.println("User Connected");
    SerialBT.println("Press enter\n");
    while (SerialBT.available() == 0);// Wait for a special command to start the calibration process
    calibrate();
  }
}


void calibrate()
{
  SerialBT.println("\nCALIBRATION\n===========");
  SerialBT.println("remove all weight from the loadcell");
  Serial.println("\nCALIBRATION\n===========");
  Serial.println("remove all weight from the loadcell");
  Serial.println("and press enter (via BT)\n");
  while (SerialBT.available()) SerialBT.read();  // Flush Bluetooth input

  SerialBT.println("and press enter\n");
  while (SerialBT.available() == 0);

  SerialBT.println("Determine zero weight offset");
  Serial.println("Determine zero weight offset");
  myScale.tare(20);  // Average 20 measurements
  uint32_t offset = myScale.get_offset();

  SerialBT.print("OFFSET: ");
  SerialBT.println(offset);
  SerialBT.println();
  Serial.print("OFFSET: ");
  Serial.println(offset);
  Serial.println();


  SerialBT.println("place a weight on the loadcell");
  Serial.println("place a weight on the loadcell");  //REGULAR SERIAL
  while (SerialBT.available()) SerialBT.read();  // Flush Bluetooth input

  SerialBT.println("enter the weight in (whole) grams and press enter");
  Serial.println("enter the weight in (whole) grams and press enter (Via BT)");//REGULAR SERIAL
  uint32_t weight = 0;
  while (SerialBT.peek() != '\n')
  {
    if (SerialBT.available())
    {
      char ch = SerialBT.read();
      if (isdigit(ch))
      {
        weight *= 10;
        weight += ch - '0';
      }
    }
  }
  SerialBT.print("WEIGHT: ");
  SerialBT.println(weight);
  Serial.print("WEIGHT: ");//REGULAR SERIAL
  Serial.println(weight);  //REGULAR SERIAL
  myScale.calibrate_scale(weight, 20);
  float scale = myScale.get_scale();

  SerialBT.print("SCALE:  ");
  SerialBT.println(scale, 6);
  Serial.print("SCALE:  "); //REGULAR SERIAL
  Serial.println(scale, 6); //REGULAR SERIAL

  SerialBT.print("\nuse \nscale.set_offset(");
  SerialBT.print(offset);
  SerialBT.print("); \nscale.set_scale(");
  SerialBT.print(scale, 6);
  SerialBT.print(");\n");

  Serial.print("\nuse \nscale.set_offset(");
  Serial.print(offset);
  Serial.print("); \nscale.set_scale(");
  Serial.print(scale, 6);
  Serial.print(");\n");
  Serial.println("in the setup of your project");
  Serial.println("\n");
}

// -- END OF FILE --
