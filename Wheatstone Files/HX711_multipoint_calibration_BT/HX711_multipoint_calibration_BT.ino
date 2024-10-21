#include <BluetoothSerial.h>
#include "HX711.h"
#include <Preferences.h>  // For storing data in flash memory

// Create HX711 object
HX711 scale;

// Bluetooth Serial object
BluetoothSerial SerialBT;

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
  // Initialize Bluetooth Serial communication
  SerialBT.begin("ESP32_LoadCell"); // Name of the Bluetooth device
  while (!SerialBT.connected(0)) {   // Wait for Bluetooth connection
    delay(100);
  }
  SerialBT.println("Bluetooth connected!");

  // Initialize the HX711 scale
  scale.begin(dataPin, clockPin);
  scale.set_scale(1.0);  // Initialize scale with a scale factor of 1

  // Initialize Preferences
  preferences.begin("calibration", false); // Namespace: "calibration", Read/Write mode

  // Check if previous calibration data exists
  if (preferences.isKey("weight0")) {
    SerialBT.println("Previous calibration data found.");
    char answer = '\0';
    while (answer != 'Y' && answer != 'y' && answer != 'N' && answer != 'n') {
      SerialBT.println("Do you want to recalibrate? (Y/N)");

      // Wait for user input
      String input = "";
      while (input.length() == 0) {
        while (SerialBT.available() == 0) {
          // wait for user input
          delay(100);
        }
        input = SerialBT.readStringUntil('\n');
        input.trim(); // Remove any whitespace
      }

      answer = input.charAt(0);
      if (answer != 'Y' && answer != 'y' && answer != 'N' && answer != 'n') {
        SerialBT.println("Invalid input. Please enter 'Y' or 'N'.");
      }
    }

    if (answer == 'Y' || answer == 'y') {
      // User wants to recalibrate
      SerialBT.println("Starting recalibration...");
      calibrateScale();
    } else {
      // User doesn't want to recalibrate
      SerialBT.println("Using existing calibration data.");
      // Load existing calibration data
      loadCalibrationData();
    }
  } else {
    SerialBT.println("No previous calibration data found.");
    // Start the calibration process
    calibrateScale();
  }
}

void loop() {
  // The main program can go here.
  // For this example, we'll just read and display the weight using the calibrated scale.
  float weight = getCalibratedWeight();
  SerialBT.print("Weight: ");
  SerialBT.print(weight, 2); // Print with 2 decimal places
  SerialBT.println(" grams");

  delay(1000); // Wait 1 second before next reading
}

void calibrateScale() {
  SerialBT.println("Starting calibration...");

  // Instruct the user
  SerialBT.println("First, ensure that the scale is empty to capture the tare value.");
  SerialBT.println("Press 'Y' to confirm that the scale is empty and proceed with tare measurement.");

  // Wait for user input to confirm tare
  char confirm = '\0';
  while (confirm != 'Y' && confirm != 'y') {
    while (SerialBT.available() == 0) {
      // Wait for input
      delay(100);
    }
    String input = SerialBT.readStringUntil('\n');
    input.trim(); // Remove any whitespace
    confirm = input.charAt(0);
    if (confirm != 'Y' && confirm != 'y') {
      SerialBT.println("Invalid input. Please press 'Y' to confirm that the scale is empty.");
    }
  }

  for (int i = 0; i < numCalibrationPoints; i++) {
    if (i == 0) {
      SerialBT.println("Leave the scale empty (Point 1 of 8).");
      calibrationWeights[i] = 0; // Tare weight is zero grams
    } else {
      SerialBT.print("Place known weight on scale (Point ");
      SerialBT.print(i + 1);
      SerialBT.println(" of 8) and enter the weight in grams:");

      // Wait for user input
      while (SerialBT.available() == 0) {
        // Wait for input
        delay(100);
      }

      // Read the weight entered by the user
      String weightInput = SerialBT.readStringUntil('\n');
      weightInput.trim(); // Remove any whitespace

      // Check if the input is a valid number
      bool validNumber = true;
      for (unsigned int idx = 0; idx < weightInput.length(); idx++) {
        if (!isDigit(weightInput.charAt(idx)) && weightInput.charAt(idx) != '.' && weightInput.charAt(idx) != '-') {
          validNumber = false;
          break;
        }
      }

      if (!validNumber) {
        SerialBT.println("Invalid input. Please enter a numeric value.");
        i--; // Repeat this iteration
        continue;
      }

      float knownWeight = weightInput.toFloat();

      // Allow 0 gram input (since we have already captured 0 at first point)
      if (knownWeight <= 0) {
        SerialBT.println("Invalid weight entered. Please enter a positive number.");
        i--; // Repeat this iteration
        continue;
      }

      calibrationWeights[i] = knownWeight;
    }

    // Perform ten raw measurements and take the average
    SerialBT.println("Performing measurements...");
    float averageReading = scale.read_average(10);

    calibrationReadings[i] = averageReading;

    // Print the result
    SerialBT.print("Calibration point ");
    SerialBT.print(i + 1);
    SerialBT.print(": Weight = ");
    SerialBT.print(calibrationWeights[i]);
    SerialBT.print(" grams, Average reading = ");
    SerialBT.println(averageReading, 2);

    // Store the calibration data in Preferences
    storeCalibrationData(i, calibrationWeights[i], calibrationReadings[i]);
  }

  // Now sort the calibration data in descending order
  sortCalibrationData();

  SerialBT.println("Calibration complete.");
  SerialBT.println("Calibration data has been saved to flash memory.");

  // Calculate and set the scale factors here using the calibration data
  calculateScaleFactor();
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
    SerialBT.print("Loaded Calibration Point ");
    SerialBT.print(i + 1);
    SerialBT.print(": Weight = ");
    SerialBT.print(calibrationWeights[i]);
    SerialBT.print(" grams, Average reading = ");
    SerialBT.println(calibrationReadings[i], 2);
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
  for (int i = 0; i < numCalibrationPoints; i++) {
    SerialBT.print("Reading: ");
    SerialBT.print(calibrationReadings[i]);
    SerialBT.print(", Weight: ");
    SerialBT.println(calibrationWeights[i]);
  }
}

void calculateScaleFactor() {
  // Compute slopes and intercepts for each range
  for (int i = 0; i < numCalibrationPoints - 1; i++) {
    float deltaReading = calibrationReadings[i + 1] - calibrationReadings[i];
    float deltaWeight = calibrationWeights[i + 1] - calibrationWeights[i];

    if (deltaReading == 0) {
      SerialBT.print("Error: Duplicate readings at calibration points ");
      SerialBT.print(i + 1);
      SerialBT.print(" and ");
      SerialBT.println(i + 2);
      slopes[i] = 0;
      intercepts[i] = 0;
    } else {
      slopes[i] = deltaWeight / deltaReading;
      intercepts[i] = calibrationWeights[i] - slopes[i] * calibrationReadings[i];
    }

    SerialBT.print("Range ");
    SerialBT.print(i + 1); 
    SerialBT.print(": Slope = ");
    SerialBT.print(slopes[i], 6);
    SerialBT.print(", Intercept = ");
    SerialBT.println(intercepts[i], 6);
  }
}

float getCalibratedWeight() {
  // Get the raw reading
  float reading = scale.read_average(10);

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

  // Use the slope and intercept of the identified range
  float weight = slopes[index] * reading + intercepts[index];

  return weight;
}