#include "HX711.h"

// Pin definitions
uint8_t dataPin  = 34;
uint8_t clockPin = 23;
uint8_t buttonPin = 4;
uint8_t ledPin = 5;

// Create an instance of the HX711 class
HX711 scale;


void setup() {
  scale.begin(dataPin, clockPin);
  // Start the serial communication with the new baud rate
  Serial.begin(115200);
  // Set the scale calibration values
  scale.set_offset(4293649106);  
  scale.set_scale(-5.752855);  
  // Set the mode to HX711_MEDAVG_MODE
  scale.set_medavg_mode();
  Serial.println(scale.get_mode());
  // Set the button pin as input
  pinMode(buttonPin, INPUT_PULLUP);
  // Set the LED pin as output
  pinMode(ledPin, OUTPUT);
}
void loop() {
  // Check if the button is pressed
  if (digitalRead(buttonPin) == LOW) {
    // Call the tare function
    Serial.println("PERFORMING TARE() PROCESS");
    scale.tare(20);
  }
  // Get the current reading
  float f = scale.get_units(3);
  // Blink the LED
  digitalWrite(ledPin, HIGH);
  // Get the raw reading
  float rawReading = scale.read();
  // Get the median average reading
  float medAvgReading = scale.read_medavg(3);
  // Get the current offset value
  long offset = scale.get_offset();
  // Print the readings and the offset value
  Serial.print(f);
  Serial.print("\t");
  Serial.print(rawReading);
  Serial.print("\t");
  Serial.print(medAvgReading);
  Serial.print("\t");
  Serial.print(offset);
  Serial.print("\t");
  Serial.println((uint32_t)offset);

  digitalWrite(ledPin, LOW);
}
