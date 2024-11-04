// #include <BluetoothSerial.h>
#include "HX711.h"
#include <Preferences.h>  // For storing data in flash memory

#define DEBUG_SORT
#define DEBUG
#define SIMPLIFY
#define SHOWREAD

// Create HX711 object
HX711 scale;

// Bluetooth Serial object
// BluetoothSerial SerialBT;

// Pin definitions for HX711
uint8_t dataPin = 34;  // Data pin for HX711
uint8_t clockPin = 23; // Clock pin for HX711

// Keypad pin definitions (Not used in this code, but defined for future use)
uint8_t keyPadPins[] = {4, 0, 2, 15};

// Create Preferences object
Preferences preferences;

// Constants
const int numCalibrationPoints = 8;  // Updated to 6 points

// Arrays to hold calibration data
float calibrationWeights[numCalibrationPoints];
float calibrationReadings[numCalibrationPoints];

// Arrays to hold slopes and intercepts for each range
float slopes[numCalibrationPoints - 1];
float intercepts[numCalibrationPoints - 1];

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);

  // Initialize Bluetooth Serial (assuming Serial is used for Bluetooth)
  // Serial.begin("ESP32test"); // Uncomment and modify if using Bluetooth
  Serial.println("StoreCalibration_8p.ino");

  // Initialize the HX711 scale
  scale.begin(dataPin, clockPin);
  scale.set_scale(1.0);  // Initialize scale with a scale factor of 1

  // Initialize preferences
  preferences.begin("calibration", false);

  // Define your calibration data
  float weights[] = {0, 1745, 4265, 6000, 8155, 9318, 10375, 11752};
  float readings[] = {-1337500, -1346021.00, -1367780.62, -1379707.87, -1382843.25, -1391240.62, -1396204.25, -1406132.75};

  // Store each calibration data point
  for (int i = 0; i < 8; i++) {
    storeCalibrationData(i, weights[i], readings[i]);
  }

  // Load and print the calibration data to verify
  loadCalibrationData();
}

void loop() {
  // For this example, we'll just read and display the weight using the calibrated scale.
  float weight = getCalibratedWeight();

  #ifndef SIMPLIFY
  Serial.print("Weight: ");
  Serial.print(weight, 2); // Print with 2 decimal places
  Serial.println(" grams");
  #else
  Serial.println(weight, 2); // Print with 2 decimal places
  #endif

  delay(1000); // Wait 1 second before next reading
}

void storeCalibrationData(int index, float weight, float reading) {
  // Store the weight and reading with unique keys
  String weightKey = "weight" + String(index);
  String readingKey = "reading" + String(index);

  preferences.putFloat(weightKey.c_str(), weight);
  preferences.putFloat(readingKey.c_str(), reading);
}

void loadCalibrationData() {
  for (int i = 0; i < numCalibrationPoints; i++) {
    String weightKey = "weight" + String(i);
    String readingKey = "reading" + String(i);

    calibrationWeights[i] = preferences.getFloat(weightKey.c_str(), 0.0);
    calibrationReadings[i] = preferences.getFloat(readingKey.c_str(), 0.0);

    // Print the loaded data
    Serial.print("Loaded Calibration Point ");
    Serial.print(i + 1);
    Serial.print(": Weight = ");
    Serial.print(calibrationWeights[i]);
    Serial.print(" grams, Average reading = ");
    Serial.println(calibrationReadings[i], 2);
  }

  // Now sort the calibration data
  sortCalibrationData();

  // Calculate slopes and intercepts
  calculateScaleFactor();
}

void sortCalibrationData() {
  // Sort the calibration data in descending order of readings
  for (int i = 0; i < numCalibrationPoints - 1; i++) {
    for (int j = i + 1; j < numCalibrationPoints; j++) {
      if (calibrationWeights[i] > calibrationWeights[j]) {
        // Swap calibrationReadings
        float tempReading = calibrationReadings[i];
        calibrationReadings[i] = calibrationReadings[j];
        calibrationReadings[j] = tempReading;

        // Swap calibrationWeights
        float tempWeight = calibrationWeights[i];
        calibrationWeights[i] = calibrationWeights[j];
        calibrationWeights[j] = tempWeight;
      }
    }
  }

  // Print the sorted values
  #ifdef DEBUG_SORT
  for (int i = 0; i < numCalibrationPoints; i++) {
    Serial.print("Weight: ");
    Serial.print(calibrationWeights[i]);
    Serial.print(", Reading: ");
    Serial.println(calibrationReadings[i]);
  }
  #endif
}

void calculateScaleFactor() {
  // Compute slopes and intercepts for each range
  for (int i = 0; i < numCalibrationPoints - 1; i++) {   
    float deltaReading = calibrationReadings[i + 1] - calibrationReadings[i];
    float deltaWeight = calibrationWeights[i + 1] - calibrationWeights[i];

    #ifdef DEBUG_SORT
    Serial.print("\nWeight: ");
    Serial.print(calibrationWeights[i]);
    Serial.print("\t");
    Serial.print(calibrationWeights[i+1]);
    Serial.print("\t");
    Serial.println(deltaWeight);

    Serial.print("Reading: ");
    Serial.print(calibrationReadings[i]);
    Serial.print("\t");
    Serial.print(calibrationReadings[i+1]);
    Serial.print("\t");
    Serial.println(deltaReading);
    #endif

    if (deltaReading == 0) {
      Serial.print("Error: Duplicate readings at calibration points ");
      Serial.print(i + 1);
      Serial.print(" and ");
      Serial.println(i + 2);
      slopes[i] = 0;
      intercepts[i] = 0;
    } else {
      slopes[i] = deltaReading / deltaWeight;
      intercepts[i] = calibrationReadings[i] - slopes[i] * calibrationWeights[i];
    }

    Serial.print("Range ");
    Serial.print(i + 1); 
    Serial.print(": Slope = ");
    Serial.print(slopes[i], 6);
    Serial.print(", Intercept = ");
    Serial.println(intercepts[i], 6);
  }
}

float getCalibratedWeight() {
  // Get the raw reading
  float reading = scale.read_average(10);

  #ifdef SHOWREAD
  Serial.print("Show Reading: ");
  Serial.print(reading);
  Serial.print("\t");
  #endif

  int index = -1;

  if (reading >= calibrationReadings[0]) {
    // Reading is higher than the highest calibration reading
    index = 0;
  } else if (reading <= calibrationReadings[numCalibrationPoints - 1]) {
    // Reading is lower than the lowest calibration reading
    index = numCalibrationPoints - 2;
  } else {
    // Reading is within the calibration range
    for (int i = 0; i < numCalibrationPoints - 1; i++) {
      if (reading <= calibrationReadings[i] && reading >= calibrationReadings[i + 1]) {
        index = i;
        break;
      }
    }
  }

  if (index == -1) {
    // This should not happen, but just in case
    index = 0;
  }

  #ifdef DEBUG
  Serial.print("Idx: ");
  Serial.print(index);
  Serial.print(", SL[]: ");
  Serial.print(slopes[index]);
  Serial.print(", Inter[]: ");
  Serial.print(intercepts[index]);
  Serial.print("\t");   
  #endif

  // Use the slope and intercept of the identified range
  float weight = (reading - intercepts[index]) / slopes[index] ;

  return weight;
}