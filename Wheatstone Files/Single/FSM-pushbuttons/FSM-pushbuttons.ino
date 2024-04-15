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
  if (millis() - lastToggleTime >= blinkIntervals[state]) {
    digitalWrite(outputPin, !digitalRead(outputPin));
    lastToggleTime = millis();
  }

  // Perform an action based on the current state

  switch (state) {
    case STATE_1:
      // Perform action for state 1
      // Serial.print("ST:100,");
      break;
    case STATE_2:
      // Perform action for state 2
      // Serial.print("ST:200,");
      scale.set_offset(0); 
      break;
    case STATE_3:
      // Perform action for state 3
      // Serial.print("ST:300,");
      scale.set_offset(4293647715); 
      break;
    case STATE_4:
      // Perform action for state 4
      Serial.print("ST:400,");
      scale.tare(10);
      state = STATE_1;
      break;
  }

  
  #define ARD_PRINT 

  if (scale.is_ready()){
    // Execute the provided code once every cycle

    Serial.print("F:");
    float f = scale.get_units(5);
    Serial.print(f);
    Serial.print(",   ");
    Serial.print("offset:");
    Serial.print(scale.get_offset()/10000.0f, 4);
    Serial.print(",   ");
    Serial.print("ST:");
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
        Serial.print("400");
        state = STATE_1;
        break;
    }
    Serial.print(",");
    Serial.println();
  }

  delay(1);
}
