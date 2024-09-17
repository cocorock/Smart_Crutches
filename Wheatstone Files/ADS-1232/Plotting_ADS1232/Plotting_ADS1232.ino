#include <EEPROM.h>
#include <ADS1232.h>

// // Define pins for the ADS-1232
#define SCALE_DOUT  10
#define SCALE_SCLK  9
#define SCALE_PDWN  11
#define SCALE_SPEED 12
#define LEDPIN 13

//Pins for ESP-32
// #define SCALE_DOUT   33
// #define SCALE_SCLK   26
// #define SCALE_PDWN   25
// #define SCALE_SPEED  32
// #define LEDPIN 2


// Struct to hold calibration data
struct Calibration {
  float scale;
  float offset;
};

// Instantiate the ADS-1232 object
ADS1232 scale;

// EEPROM addresses to store calibration data
const int EEPROM_OFFSET_ADDR = 0;
const int EEPROM_SCALE_ADDR = EEPROM_OFFSET_ADDR + sizeof(float);

Calibration calibrationData;

// Function to read calibration data from EEPROM
void readCalibrationFromEEPROM(Calibration &cal) {
  EEPROM.get(EEPROM_OFFSET_ADDR, cal.offset);
  EEPROM.get(EEPROM_SCALE_ADDR, cal.scale);
}

void setup() {
  pinMode(LEDPIN, OUTPUT);
  pinMode(SCALE_SPEED, OUTPUT);

  Serial.begin(115200);
  scale.begin(SCALE_DOUT, SCALE_SCLK, SCALE_PDWN, SCALE_SPEED, FAST);

  // // Read existing calibration data from EEPROM
  // readCalibrationFromEEPROM(calibrationData);
  // scale.set_offset(calibrationData.offset);
  // scale.set_scale(calibrationData.scale);

  delay(500);
  scale.set_scale(-3.28);
  
  // Print existing calibration values
  Serial.println("Existing Calibration Values:");
  Serial.print("Offset: ");
  Serial.println(calibrationData.offset);
  Serial.print("Scale: ");
  Serial.println(calibrationData.scale);

  
  
  delay(500);

  scale.tare(20); // Set new Tare value instead of the stored value
  calibrationData.offset = scale.get_offset();
  Serial.print("Offset: ");
  Serial.println(calibrationData.offset);

  //digitalWrite(SCALE_SPEED, HIGH);
}

void loop() {
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  float weight;
  ERROR_t err = scale.get_units(weight, 1); // Get average weight from 10 readings

  if (err == NoERROR) {
    // Print the weight
    // Serial.print("Weight: ");
    Serial.println(weight);
    // Serial.println(" g");
  } else {
    Serial.println("Error reading weight");
  }

  delay(1000);
}
