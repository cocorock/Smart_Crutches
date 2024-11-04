#include <Arduino.h>
#include "HX711.h"

// Define whether to use Bluetooth Serial or standard Serial
#define USE_BLUETOOTH true // Set to true to use Bluetooth serial

#if USE_BLUETOOTH
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

// Pin definitions for HX711
uint8_t dataPin = 34; // Data pin for HX711
uint8_t clockPin = 23; // Clock pin for HX711

HX711 scale;

void setup() {
  // Initialize serial ports
  Serial.begin(115200);

  Serial.println("Calibration_sampling_BT.ino");

#if USE_BLUETOOTH
  SerialBT.begin("ESP32_HX711"); // Name of the Bluetooth device
#endif

  // Initialize the HX711 scale
  scale.begin(dataPin, clockPin);

  // Wait until HX711 is ready
  scale.wait_ready();
}

void loop() {
  // Wait until HX711 is ready to read
  scale.wait_ready();

  // Perform 10 raw reads and compute the average
  float average = scale.read_average(18);

  // Get milliseconds since boot
  uint32_t elapsedTime = millis();

  // Format the milliseconds and the average value with commas
  String timeString = formatWithCommas(elapsedTime);
  String valueString = formatWithCommas((long)average); // Casting average to long for formatting

  // Prepare the message
  String message = timeString + "\t" + valueString;

  // Send the message over the appropriate serial port
#if USE_BLUETOOTH
  SerialBT.println(message);
#else
  Serial.println(message);
#endif
}

// Function to format numbers with commas separating thousands
String formatWithCommas(long number) {
  bool isNegative = false;
  if (number < 0) {
    isNegative = true;
    number = -number;
  }

  String numStr = String(number);
  String formattedStr = "";
  int len = numStr.length();
  int commaCounter = 0;

  for (int i = len - 1; i >= 0; i--) {
    formattedStr = numStr.charAt(i) + formattedStr;
    commaCounter++;
    if (commaCounter == 3 && i != 0) {
      formattedStr = "," + formattedStr;
      commaCounter = 0;
    }
  }

  if (isNegative) {
    formattedStr = "-" + formattedStr;
  }

  return formattedStr;
}