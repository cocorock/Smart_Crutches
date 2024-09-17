#include <EEPROM.h>
#include <ADS1232.h>

// Define pins for the ADS-1232
// #define SCALE_DOUT  4
// #define SCALE_SCLK  9
// #define SCALE_PDWN  11
// #define SCALE_SPEED 12
// #define LEDPIN 13

//Pins for ESP-32
#define SCALE_DOUT   33
#define SCALE_SCLK   26
#define SCALE_PDWN   25
#define SCALE_SPEED  32
#define LEDPIN 2


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

// Function to write calibration data to EEPROM
void writeCalibrationToEEPROM(const Calibration &cal) {
  EEPROM.put(EEPROM_OFFSET_ADDR, cal.offset);
  EEPROM.put(EEPROM_SCALE_ADDR, cal.scale);
}

// Function to read calibration data from EEPROM
void readCalibrationFromEEPROM(Calibration &cal) {
  EEPROM.get(EEPROM_OFFSET_ADDR, cal.offset);
  EEPROM.get(EEPROM_SCALE_ADDR, cal.scale);
}

void setup() {
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(115200);
  scale.begin(SCALE_DOUT, SCALE_SCLK, SCALE_PDWN, SCALE_SPEED, SLOW);

  // Read existing calibration data from EEPROM
  readCalibrationFromEEPROM(calibrationData);
  scale.set_offset(calibrationData.offset);
  scale.set_scale(calibrationData.scale);

  // Print existing calibration values
  Serial.println("\n\nExisting Calibration Values:");
  Serial.print("Offset: ");
  Serial.println(calibrationData.offset);
  Serial.print("Scale: ");
  Serial.println(calibrationData.scale);
  Serial.println("Starting calibration...");
}

void loop() {
  // Clear the serial buffer
  while (Serial.available() > 0) {
    Serial.read();
  }
  // Step 1: Find offset
  Serial.println("\n\n1. Remove any weight from the scale and press any key to start offset calibration.");
  while (!Serial.available());
  Serial.read();
  scale.tare(20); // Set offset by averaging 20 readings with no weight
  calibrationData.offset = scale.get_offset();
  Serial.print("Offset: ");
  Serial.println(calibrationData.offset);

  // Clear the serial buffer again
  while (Serial.available() > 0) {
    Serial.read();
  }
  // Step 2: Find scale factor
  Serial.println("\n\n2. Place a known weight on the scale and enter the weight in grams:");
  while (!Serial.available());
  float knownWeight = Serial.parseFloat();
  Serial.print("Known weight: ");
  Serial.println(knownWeight);

  // long temp;
  // scale.read(temp, true);
  // scale.read(temp, true);

  // float rawValue;
  // scale.read_average(rawValue, 10, true); // Get average of 20 readings
  // Serial.print("\n\n>Comp read_average 10: ");
  // Serial.println(rawValue);
  // calibrationData.scale = (rawValue - calibrationData.offset) / knownWeight;
  // scale.set_scale(calibrationData.scale);
  // Serial.print("Scale: ");
  // Serial.println(scale.get_scale());

  long R1;
  scale.read(R1, true);
  scale.read(R1, true);
  Serial.print("R1: ");
  Serial.println(R1);

  float RAW;
  scale.read_average(RAW, 10, true);
  Serial.print("RAW: ");
  Serial.println(RAW);
  float diff = (float)RAW - calibrationData.offset;
  Serial.print("diff: ");
  Serial.println(diff);

  float scl = diff / knownWeight;
  Serial.print("scl: ");
  Serial.println(scl);  
  calibrationData.scale = scl;

  // Save the calibration data to EEPROM
  writeCalibrationToEEPROM(calibrationData);
  Serial.println("\nCalibration data saved to EEPROM.");

  Serial.println("Calibration complete.");

  // Toggle LED to indicate calibration cycle completion
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
}