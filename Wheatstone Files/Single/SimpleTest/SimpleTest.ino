// FILE: HX_plotter.ino
// AUTHOR: Rob Tillaart
// PURPOSE: HX711 demo
// URL: https://github.com/RobTillaart/HX711
String file_name = "Programa:\n\t SimpleTest.ino"; 

#include "HX711.h"
#include <EEPROM.h>

HX711 scale;

const uint8_t dataPin = 34;
const uint8_t clockPin = 23;

// EEPROM start address for calibration data
const int calibrationAddress = 321;

void setup() {
  Serial.begin(250000);
  Serial.println(file_name);
  scale.begin(dataPin, clockPin);

  // Initialize EEPROM with a size of 512 bytes
  EEPROM.begin(512);

  // Load calibration from EEPROM
  loadFromMemory();
}

void loop() {
  if (scale.is_ready()) {
    float reading = scale.get_units(1);
    Serial.println(String(esp_timer_get_time()/10e5) + ",\t" + String(reading));
  }
  delay(1); // adjust the delay as needed
}

// Function to read calibration data from EEPROM
void readCalibrationFromEEPROM(float &scaleValue, float &offsetValue) {
  EEPROM.get(calibrationAddress, scaleValue);
  EEPROM.get(calibrationAddress + sizeof(float), offsetValue);
}

// Function to write calibration data to EEPROM
void writeCalibrationToEEPROM(float scaleValue, float offsetValue) {
  EEPROM.put(calibrationAddress, scaleValue);
  EEPROM.put(calibrationAddress + sizeof(float), offsetValue);
  EEPROM.commit();  // Save changes to EEPROM
}

// Subroutine to handle all reading from memory
void loadFromMemory() {
  float scaleValue, offsetValue;
  readCalibrationFromEEPROM(scaleValue, offsetValue);

  // Check if the EEPROM contains valid data
  if (isnan(scaleValue) || scaleValue == 0) {
    scaleValue = -5.752855;  // Default scale value
    Serial.println("No Calibration Found!");
    Serial.println("Scale Value set: " + String(scaleValue));
  }else{
    Serial.println("Scale Value set From Mem: " + String(scaleValue));
  }
  scale.set_scale(scaleValue);

  if (isnan(offsetValue)) {
    delay(500);
    scale.tare(20);
    delay(500);
    Serial.println("Scale tared!");
  }else{
    Serial.println("Offset Value set From Mem: " + String(offsetValue));
    scale.set_offset(offsetValue);
  }
}