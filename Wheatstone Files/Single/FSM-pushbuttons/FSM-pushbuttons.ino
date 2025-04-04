#include "HX711.h"

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
uint8_t readings = 1;

// Define the states
enum State {STATE_1, STATE_2, STATE_3, STATE_4};
State state = STATE_1;

// Define the blink intervals for each state (in milliseconds)
const int blinkIntervals[] = {500, 500, 250, 125};

// Keep track of the last time the LED state was toggled
unsigned long lastToggleTime = 0;

void setup() {
  // Initialize the button pins as inputs
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Initialize the output pin as output
  pinMode(outputPin, OUTPUT);

  //set mode
  scale.set_average_mode();
  Serial.print(scale.get_mode());

  // Initialize the HX711 scale
  Serial.begin(115200);
  scale.begin(dataPin, clockPin);
  // scale.set_scale(-4.868363);
  // scale.set_scale(-6.450881);
  scale.set_scale(-6.4);
  scale.set_offset(4293647715);
  delay(50);
  scale.tare(20);
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
      scale.tare(10);
      delay(500);
      break;
  }

  
  #define ARD_PRINT 

  if (scale.is_ready()){
    // Execute the provided code once every cycle

    // Get the current reading
    float f = scale.get_units(readings);

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
  }

  delay(1);
}
