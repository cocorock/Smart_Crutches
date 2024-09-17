#include <EEPROM.h>
#include <ADS1232.h>

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
const int EEPROM_SIZE = 512; // Define the size of EEPROM
const int EEPROM_OFFSET_ADDR = 0;
const int EEPROM_SCALE_ADDR = EEPROM_OFFSET_ADDR + sizeof(float);

Calibration calibrationData;

// Function to write calibration data to EEPROM
void writeCalibrationToEEPROM(const Calibration &cal) {
  EEPROM.put(EEPROM_OFFSET_ADDR, cal.offset);
  EEPROM.put(EEPROM_SCALE_ADDR, cal.scale);
  EEPROM.commit(); // Commit changes to EEPROM
}

// Function to read calibration data from EEPROM
void readCalibrationFromEEPROM(Calibration &cal) {
  EEPROM.get(EEPROM_OFFSET_ADDR, cal.offset);
  EEPROM.get(EEPROM_SCALE_ADDR, cal.scale);
}

void setup() {
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE); // Initialize EEPROM with the defined size
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

  // Step 1: Find offset
  Serial.println("Remove any weight from the scale and press any key to start offset calibration.");
  while (!Serial.available());
  Serial.read();
  scale.tare(20); // Set offset by averaging 10 readings with no weight
  calibrationData.offset = scale.get_offset();
  Serial.print("Offset: ");
  Serial.println(calibrationData.offset);

  // Step 2: Find scale factor
  Serial.println("Place a known weight on the scale and enter the weight in grams:");
  while (!Serial.available());
  float knownWeight = Serial.parseFloat();
  Serial.print("Known weight: ");
  Serial.println(knownWeight);

  long temp;
  scale.read(temp, true);
  scale.read(temp, true);

  float rawValue;
  scale.read_average(rawValue, 20, true); // Get average of 10 readings
  Serial.print("C read_average 20: ");
  Serial.println(rawValue);
  calibrationData.scale = (rawValue - calibrationData.offset) / knownWeight;
  scale.set_scale(calibrationData.scale);
  Serial.print("Scale: ");
  Serial.println(scale.get_scale());

  long RAW;
  scale.read(RAW, true);
  scale.read(RAW, true);
  Serial.print("RAW: ");
  Serial.println(RAW);
  float diff = (float)RAW - calibrationData.offset;
  Serial.print("diff: ");
  Serial.println(diff);

  float scl = diff / knownWeight;
  Serial.print("scl: ");
  Serial.println(scl);  

  // Save the calibration data to EEPROM
  writeCalibrationToEEPROM(calibrationData);
  Serial.println("\nCalibration data saved to EEPROM.");

  Serial.println("Calibration complete.");
}

void loop() {
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  float weight;
  scale.get_units(weight, 10); // Get average weight from 10 readings

  // Print the weight
  Serial.println(weight);

  delay(500);
}