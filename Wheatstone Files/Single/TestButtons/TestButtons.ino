// Define the pin numbers
const int buttonPins[] = {4, 0, 2, 15};
const int numButtons = sizeof(buttonPins)/sizeof(int);

// Define the output pin
const int outputPin = 5;  // Built-in LED of ESP32 TTGO T18_3.0

void setup() {
  // Initialize the button pins as inputs
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Initialize the output pin as output
  pinMode(outputPin, OUTPUT);

  // Initialize the Serial Monitor
  Serial.begin(115200);
}

void loop() {
  // Check each button in turn
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {
      // If the button is pressed, print a message
      Serial.print("Button on pin ");
      Serial.print(buttonPins[i]);
      Serial.println(" is pressed.");

      // Blink the LED the appropriate number of times
      for (int j = 0; j <= i; j++) {
        digitalWrite(outputPin, HIGH);
        delay(100);
        digitalWrite(outputPin, LOW);
        delay(100);
      }

      delay(300);  // Debounce delay
    }
  }
}
