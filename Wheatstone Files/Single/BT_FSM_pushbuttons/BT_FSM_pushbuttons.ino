
#include "HX711.h"
#include "BluetoothSerial.h" // Include the BluetoothSerial library
#include "esp_system.h"  // Include the ESP32 system header

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BT_PRINT
// #define ARD_PRINT
#define BETTER_PLOTTER

#ifdef BT_PRINT
  BluetoothSerial SerialBT; // Create an instance of BluetoothSerial called "SerialBT"
#endif

// Define the pin numbers
const int buttonPins[] = {4, 0, 2, 15};
const int numButtons = sizeof(buttonPins)/sizeof(int);

// Define the output pin
const int outputPin = 5;

// Define the HX711 data and clock pins
uint8_t dataPin  = 34;
uint8_t clockPin = 23;

// Create an instance of HX711
HX711 scale;
// Initialize the HX711 scale
uint8_t readings = 1;

// Define the states
enum State {STATE_1, STATE_2, STATE_3, STATE_4};
State state = STATE_1;

// Define the blink intervals for each state (in milliseconds)
//const int blinkIntervals[] = {500, 500, 250, 125};

// Keep track of the last time the LED state was toggled
unsigned long lastToggleTime = 0;

void setup() {
  scale.begin(dataPin, clockPin);
  // Initialize the button pins as inputs
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Initialize the output pin as output
  pinMode(outputPin, OUTPUT);

    uint64_t chipid = 102390;
  #ifdef BT_PRINT
    // Initialize the Bluetooth Serial port
    chipid = ESP.getEfuseMac(); // Get the MAC address of the ESP32
    String deviceName = "Crutch-BT-" + String((uint32_t)chipid); // Create a unique device name
    SerialBT.begin(deviceName); // Start the Bluetooth with the unique device name
  #endif

  #if defined(ARD_PRINT) || defined(BETTER_PLOTTER)
    Serial.begin(115200);
    if (Serial.available()) { } 
  #endif

  
  if (chipid == 123380739632932){ 
    scale.set_offset(4293656302);  //Crutch 1
    scale.set_scale(-5.752855);   //Crutch 1
    Serial.println("Crutch 1");
  }else if(chipid == 1160277796){
    scale.set_offset(4293032722);  //Crutch 2
    scale.set_scale(-5.639015);    //Crutch 2
    Serial.println("Crutch 2");
  }else{
    Serial.println("===================    Error 1    ====================");
    Serial.println(chipid);
  }
  
 
  // scale.tare(20);
  //set mode
  scale.set_medavg_mode();
  Serial.print("MODE:");
  Serial.println(scale.get_mode());
  Serial.print("FASTPROCESS:");
  // Serial.println(scale._fastProcessor);

}

void loop() {
  // Check each button in turn
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {
      // If the button is pressed, change the state
      state = static_cast<State>(i);
      break;
    }
  }

  // If enough time has passed since the last toggle, toggle the LED state
  if (millis() - lastToggleTime >= (readings*100) ) { // 1 sample period at @10Hz times readings
    digitalWrite(outputPin, !digitalRead(outputPin));
    lastToggleTime = millis();
  }

  // Perform an action based on the current state
  switch (state) {
    case STATE_1:
      // default state always do
      break;
    case STATE_2:
      // Increase number of readins per measurement
      readings++;
      delay(300); 
      break;
    case STATE_3:
      // reset number of readings
      readings=1; 
      break;
    case STATE_4:
      // tare the scale

      float t = scale.get_tare();
      long of = scale.get_offset();
      float sc = scale.get_scale();
      uint8_t g = scale.get_gain();
      float v = scale.get_value();
      float avg = scale.read_average(5);
      uint32_t l =  scale.read();

      #ifdef BT_PRINT
        SerialBT.println(scale.get_mode());
        SerialBT.print("get_offset: ");
        SerialBT.println(of);
      #endif
      #ifdef BETTER_PLOTTER
        Serial.print(0);   // second variable is sin(t)
        Serial.print("\t");
        Serial.print(100*readings);   // second variable is sin(t)
        Serial.print("\t");      // this last "\t" isn't required, but doesn't hurt
        Serial.print(100*state); // third varible is cos(t). make sure to finish with a println!
        
        Serial.print("\t");
        Serial.print(scale.get_mode());
        Serial.print(" - offset: ");
        Serial.print(of);
        Serial.print("  uint32_t: ");
        Serial.println((uint32_t)of);
      #endif
      delay(1000);
      scale.tare(20);
      delay(500);
      break;
  }

  if (scale.is_ready()){
    // Execute the provided code once every cycle

    // Get the current reading
    float f = scale.get_units(readings);

    #ifdef ARD_PRINT
    Serial.print("F:"); //print Force Measurement
    Serial.print(f);
    Serial.print(",   ");
    Serial.print("R:");
    Serial.print(readings); //number of readings
    Serial.print(",   ");
    Serial.print("ST:");  //State of the FSM
    switch (state) {
      case STATE_1:
        Serial.print("100");
        break;
      case STATE_2:
        Serial.print("200");
        state = STATE_1;
        break;
      case STATE_3:
        Serial.print("300");
        state = STATE_1;
        break;
      case STATE_4:
        Serial.print("500");
        state = STATE_1;
        break;
    }
    Serial.print(",");
    Serial.println();
    #endif

    #ifdef BETTER_PLOTTER
      // Serial.print(millis()/1000.0);         // first variable is program time in seconds. This can be plotted on an x-axis!
      // Serial.print("\t");
      Serial.print(f);   // second variable is sin(t)
      Serial.print("\t");
      Serial.print(100*readings);   // second variable is sin(t)
      Serial.print("\t");      // this last "\t" isn't required, but doesn't hurt
      Serial.println(100*state); // third varible is cos(t). make sure to finish with a println!
      state = STATE_1;
    #endif

    #ifdef BT_PRINT
    // Serial.print(millis()/1000.0);         // first variable is program time in seconds. This can be plotted on an x-axis!
      // Serial.print("\t");
      SerialBT.print(f);   // second variable is sin(t)
      SerialBT.print("\t");
      SerialBT.print(100*readings);   // second variable is sin(t)
      SerialBT.print("\t");      // this last "\t" isn't required, but doesn't hurt
      SerialBT.println(100*state); // third varible is cos(t). make sure to finish with a println!
      state = STATE_1;
    #endif
  }

  delay(1);
}
