#include <HX711.h>

// Pins for HX711 module and LED
const uint8_t dataPin = 34;
const uint8_t clockPin = 23;
const uint8_t ledPin = 5; // Pin for the LED

// Calibration parameters
const long offset = 4293656302; // Set your actual offset value
const float scale_factor = -5.752855;  // Set your actual scale value

// Number of measurements
const int numMeasurements = 100;

// Create an instance of the HX711 library
HX711 loadCell;

void setup() {
  Serial.begin(115200); // Increase baud rate for ESP32
  loadCell.begin(dataPin, clockPin);
  loadCell.set_offset(offset);
  loadCell.set_scale(scale_factor);

  pinMode(ledPin, OUTPUT); // Set LED pin as output
  digitalWrite(ledPin, LOW); // Turn off LED initially

  // Print the gain value during setup
  Serial.print("HX711 Gain: ");
  Serial.println(loadCell.get_gain());
  Serial.print("offset: ");
  Serial.println(offset);
  Serial.print("offset: ");
  Serial.println(offset,BIN);
}

void loop() {
  // Initialize variables for statistics
  double sum = 0.0;
  double sumSquared = 0.0;

  // Take 1000 measurements
  for (int i = 0; i < numMeasurements; ++i) {
    long rawValue = loadCell.read();
    double weight = loadCell.get_units(10); // Get weight in grams (adjust the argument as needed)
    sum += weight;
    sumSquared += weight * weight;

    // Toggle the LED between each sampling
    digitalWrite(ledPin, HIGH); // Turn on LED
    delay(10); // LED on time
    digitalWrite(ledPin, LOW); // Turn off LED
    delay(10); // LED off time
  }

  // Calculate mean and variance
  double mean = sum / numMeasurements;
  double variance = (sumSquared / numMeasurements) - (mean * mean);

  // Check if measurements follow a Gaussian distribution
  // You can set a threshold for normality based on your requirements
  if (variance < 100.0) {
    Serial.println("Measurements appear to follow a Gaussian distribution.");
  } else {
    Serial.println("Measurements do not follow a Gaussian distribution.");
  }

  // Optional: Print mean and variance
  Serial.print("Mean weight: ");
  Serial.print(mean);
  Serial.print(" grams, Variance: ");
  Serial.println(variance);

  // Wait before taking another set of measurements
  delay(5000);
}
