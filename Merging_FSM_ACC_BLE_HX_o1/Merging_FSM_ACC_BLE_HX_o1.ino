// Include necessary libraries
#include <HX711.h>                       // Library for HX711 load cell amplifier
#include "BluetoothSerial.h"             // Include the BluetoothSerial library
#include "esp_system.h"                  // Include the ESP32 system header
#include <Adafruit_Sensor_Calibration.h> // For sensor calibration
#include <Adafruit_AHRS.h>               // For AHRS systems (Madgwick/Mahony filter)
#include <EEPROM.h>                      // Include EEPROM library for saving calibration data
#include <Wire.h>                        // Include Wire library for I2C communication

//-------------------- Global Variables and Definitions ------------------------

// Program file name
String file_name = "Programa: Merging_FSM_ACC_BLE_HX_o1.ino";

//-------------------- Bluetooth ------------------------
#define BT_PRINT        // Define to enable Bluetooth printing
#define BETTER_PLOTTER  // Define to enable Serial Port printing

// Bluetooth setup
#ifdef BT_PRINT
  // Ensure Bluetooth is enabled in the config
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error ERROR 101: Bluetooth is not enabled! Please run `make menuconfig` and enable it
  #endif

  #if !defined(CONFIG_BT_SPP_ENABLED)
    #error ERROR 102: Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
  #endif

  // Create an instance of BluetoothSerial called "SerialBT"
  BluetoothSerial SerialBT;
#endif

//-------------------- Accelerometer ------------------------
// Declare pointers to sensors (must be before including NXP_FXOS_FXAS.h)
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Include the library for the NXP sensors (must be after declaring the sensors)
#include "NXP_FXOS_FXAS.h" // THIS LIBRARY HAS TO BE INCLUDED AFTER DECLARING THE Adafruit Sensors

// AHRS filter setup
#define FILTER_UPDATE_RATE_HZ 50    // Filter update rate in Hz
#define PRINT_EVERY_N_UPDATES 10    // Print every n updates
uint32_t timestamp;                 // Timestamp for filter updates

// Uncomment the filter you want to use (Madgwick is faster than NXP, Mahony is fastest/smallest)
//Adafruit_NXPSensorFusion filter; // Slowest
Adafruit_Madgwick filter;           // Faster than NXP
//Adafruit_Mahony filter;           // Fastest/smallest

// Sensor calibration storage (EEPROM or SD card)
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

//-------------------- FSM and Digital I/O ------------------------
// Define the pins for the keypad buttons
const int keyPadPins[] = { 4, 0, 2, 15 };
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;                // Pin for the LED
uint8_t  loopDelay = 2;

//-------------------- HX711 Load Cell Amplifier ------------------------
HX711 scale;                         // Create instance of HX711
uint8_t dataPin = 34;                // Data pin for HX711
uint8_t clockPin = 23;               // Clock pin for HX711
uint8_t readingsNo = 1;              // Number of readings per measurement
const int calibrationAddress = 321;  // EEPROM address for calibration data

//-------------------- State Machine Definitions ------------------------
// Define the states
enum State {
  STATE_1,   // Default state
  STATE_2,   // Increase number of readings per measurement
  STATE_3,   // Reset number of readings
  STATE_4    // Tare the scale
};
State state = STATE_1;               // Initialize the state

//-------------------- Setup Function ------------------------
void setup() {
  //-------------------- Serial Communication ------------------------
  #ifdef BETTER_PLOTTER
    Serial.begin(115200);            // Start Serial communication at baud rate 115200
    while (!Serial) {}               // Wait for Serial port to be available
    Serial.println(file_name);       // Print the program file name
  #endif

  //-------------------- Digital Input/Output ------------------------
  // Initialize keypad pins as input with pull-up resistors
  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  // Initialize LED pin as output
  pinMode(ledPin, OUTPUT);

  //-------------------- Bluetooth Initialization ------------------------
  #ifdef BT_PRINT
    // Generate a unique device name using the ESP32 MAC address
uint64_t chipid = ESP.getEfuseMac();
String chipIdString = String((uint32_t)(chipid & 0xFFFFFFFF), HEX);
String deviceName = "Calib-Crutch-BT-" + chipIdString;  // Create a unique device name
SerialBT.begin(deviceName);                       // Start Bluetooth with the unique device name
  #endif

  #ifdef USE_PIN
    SerialBT.setPin(pin);
    printMessageLn("Using PIN");
  #endif

  //-------------------- Accelerometer Initialization ------------------------
  if (!cal.begin()) {
    printMessageLn("ERROR 201: Failed to initialize calibration helper");
  } else if (!cal.loadCalibration()) {
    printMessageLn("No calibration loaded/found");
  } else {
    printCalibration();  // Print calibration data
    delay(3000);
  }

  if (!init_sensors()) {
    printMessageLn("ERROR 202: Failed to find sensors");
    while (1) delay(10);  // Stop if sensors are not found
  }

  // Print sensor details
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();                   // Setup sensors
  filter.begin(FILTER_UPDATE_RATE_HZ);  // Initialize the filter
  timestamp = millis();                 // Initialize timestamp

  Wire.setClock(400000);             // Set I2C clock to 400KHz

  //-------------------- HX711 Initialization ------------------------
  scale.begin(dataPin, clockPin);    // Initialize the HX711
  EEPROM.begin(512);                 // Initialize EEPROM with a size of 512 bytes

  // Load calibration from EEPROM
  loadFromMemory();

  printMessageLn("------------ HX711 --------------");
  scale.set_average_mode();          // Set mode to average
  printMessage("MODE: ");
  printMessageLn(getModeNameHX711(scale.get_mode()));
  printMessage("GAIN: ");
  printMessageLn(getGainNameHX711(scale.get_gain()));
}

//-------------------- Main Loop Function ------------------------
//----------------------------------------------------------------
void loop() {
  static uint8_t counter = 0;        // Counter for print updates
  bool read_flag = false;            // Flag to read sensors
  bool print_flag = false;           // Flag to print data
  float gx, gy, gz;                  // Variables for gyro readings

  //------------------- Finite State Machine (FSM) -----------------------------
  // Check each button for press
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(keyPadPins[i]) == LOW) {
      // If the button is pressed, change the state
      state = static_cast<State>(i);
      // de                            lay(50);  // Debounce delay
      break;
    }
  }

  // Handle the state actions
  switch (state) {
    case STATE_1:
      // Default state, nothing to do
      break;
    case STATE_2:
      // Increase number of readings per measurement
      readingsNo++;
      delay(300);  // Prevent rapid state change
      state = STATE_1;  // Return to default state
      break;
    case STATE_3:
      // Reset number of readings to 1
      readingsNo = 1;
      state = STATE_1;  // Return to default state
      break;
    case STATE_4:
      // Print scale parameters and tare the scale
      printScaleParams();
      delay(500);
      scale.tare(20);  // Tare the scale with 20 readings
      delay(500);
      state = STATE_1;  // Return to default state
      break;
  }

  //-------------------- Sensor Readings and Filter Update ------------------------
  if ((millis() - timestamp) >= (1000 / FILTER_UPDATE_RATE_HZ)) {
    read_flag = true;
    timestamp = millis();
  }

  if (++counter >= PRINT_EVERY_N_UPDATES) {
    print_flag = true;
    digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED state
    counter = 0;  // Reset counter
  }

  if (read_flag) {
    read_flag = false;

    // Read the motion sensors
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);       // Get accelerometer event
    gyroscope->getEvent(&gyro);            // Get gyroscope event
    magnetometer->getEvent(&mag);          // Get magnetometer event

    #if defined(AHRS_DEBUG_OUTPUT)
      printMessage("I2C took "); printMessage(String(millis() - timestamp));  printMessageLn(" ms");
    #endif

    // Calibrate sensor readings
    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // Gyroscope needs to be converted from Rad/s to Degree/s
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    #if defined(AHRS_DEBUG_OUTPUT)
      printMessage("Update took ");
      printMessage(String(millis() - timestamp));
      printMessageLn(" ms");

      printMessage("Raw: ");
      printMessage(String(accel.acceleration.x, 4)); printMessage(", ");
      printMessage(String(accel.acceleration.y, 4)); printMessage(", ");
      printMessage(String(accel.acceleration.z, 4)); printMessage(", ");
      printMessage(String(gx, 4)); printMessage(", ");
      printMessage(String(gy, 4)); printMessage(", ");
      printMessage(String(gz, 4)); printMessage(", ");
      printMessage(String(mag.magnetic.x, 4)); printMessage(", ");
      printMessage(String(mag.magnetic.y, 4)); printMessage(", ");
      printMessage(String(mag.magnetic.z, 4)); printMessageLn("");
    #endif
  }

  if (print_flag) {
    print_flag = false;

    // Print force and orientation data
    printForce_Orientation(filter);

    // printMessage("\n");  // New line

    // Uncomment if you want to print quaternion
    // printQuaternion(filter);

    #if defined(AHRS_DEBUG_OUTPUT)
      printMessage("Took "); printMessage(String(millis() - timestamp, 4)); printMessageLn(" ms");
    #endif
  }

  // Pass through data from Bluetooth to Serial if both are defined
  #if defined(BETTER_PLOTTER) && defined(BT_PRINT)
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
  #endif

  // loopDelay = 1;
  delay(loopDelay);  // Small delay to prevent watchdog resets
}

//-------------------- Helper Functions ------------------------

// Function to print messages over Bluetooth and/or Serial Port
void printMessage(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.print(message);       // Print over Bluetooth
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.print(message);           // Print over Serial Port
  #endif
}

// Function to print messages with newline over Bluetooth and/or Serial Port
void printMessageLn(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.println(message);     // Print over Bluetooth
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.println(message);         // Print over Serial Port
  #endif
}

// Function to print calibration data
void printCalibration() {
  printMessageLn("Initialization or Calibration loaded/found successfully");
  printMessageLn("Calibrations found: ");
  printMessage("\tMagnetic Hard Offset: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.mag_hardiron[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tMagnetic Soft Offset: ");
  for (int i = 0; i < 9; i++) {
    printMessage(String(cal.mag_softiron[i]));
    if (i != 8) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tMagnetic Field Magnitude: ");
  printMessageLn(String(cal.mag_field));
  printMessage("\tGyro Zero Rate Offset: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.gyro_zerorate[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tAccel Zero G Offset: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.accel_zerog[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
}

// Function to print scale parameters
void printScaleParams() {
  long offset = scale.get_offset();
  printMessage("State: " + String(state));
  printMessage("\tReadings: " + String(readingsNo));
  printMessage("\tScale Mode: " + getModeNameHX711(scale.get_mode()));
  printMessage("\tOffset: " + String(offset));
  printMessageLn("");
}

// Function to print force and orientation data
void printForce_Orientation(Adafruit_Madgwick filter) {
  float roll, pitch, heading;
  float force;

  // Read force from scale
  if (scale.is_ready()) {
    force = scale.get_units(readingsNo);  // Get the current measurement from the HX711
  } else {
    printMessage("Error 302: Scale is not ready (delay: ");
    printMessage(String(loopDelay));
    printMessage("\t ms:");
    printMessage(String(millis()));
    printMessageLn(")");
    // force = 0;  // Set force to zero if scale is not ready
  }

  // Get the orientation from the filter
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  // Print data
  // printMessage("Time: " + String(millis()));
  // printMessage("\tF: " + String(force));
  // printMessage("\tOri: ");
  // printMessage("Heading: " + String(heading, 4) + ", ");
  // printMessage("Pitch: " + String(pitch, 4) + ", ");
  // printMessage("Roll: " + String(roll, 4));
  // printMessageLn("");
  
  printMessage(String(millis()));
  printMessage("\tF:");
  printMessage(String(force));
  printMessage(",\tOr: ,");
  printMessage("H:" + String(heading, 2) + ", ");
  printMessage("P:" + String(pitch, 2) + ", ");
  printMessageLn("R:" + String(roll, 2));

}

// Function to print quaternion data
void printQuaternion(Adafruit_Madgwick filter) {
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  printMessage("Quaternion: ");
  printMessage(String(qw, 4));
  printMessage(", ");
  printMessage(String(qx, 4));
  printMessage(", ");
  printMessage(String(qy, 4));
  printMessage(", ");
  printMessage(String(qz, 4));
  printMessageLn("");
}

// Function to get the mode name of HX711
String getModeNameHX711(uint8_t mode) {
  switch (mode) {
    case HX711_AVERAGE_MODE:
      return "Average Mode";
    case HX711_MEDIAN_MODE:
      return "Median Mode";
    case HX711_MEDAVG_MODE:
      return "MedAvg Mode";
    case HX711_RUNAVG_MODE:
      return "RunAvg Mode";
    case HX711_RAW_MODE:
      return "Raw Mode";
    default:
      return "Unknown Mode";
  }
}

// Function to get the gain name of HX711
String getGainNameHX711(uint8_t gain) {
  switch (gain) {
    case HX711_CHANNEL_A_GAIN_128:
      return "Channel A Gain 128";
    case HX711_CHANNEL_A_GAIN_64:
      return "Channel A Gain 64";
    case HX711_CHANNEL_B_GAIN_32:
      return "Channel B Gain 32";
    default:
      return "Unknown Gain";
  }
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

// Function to load calibration data from memory
void loadFromMemory() {
  float scaleValue, offsetValue;
  readCalibrationFromEEPROM(scaleValue, offsetValue);

  // Check if the EEPROM contains valid data
  if (isnan(scaleValue) || scaleValue == 0) {
    scaleValue = -5.752855;  // Default scale value
    printMessageLn("No Calibration Found!");
    printMessageLn("Scale Value set: " + String(scaleValue));
  } else {
    printMessageLn("Scale Value set From Memory: " + String(scaleValue));
  }
  scale.set_scale(scaleValue);

  if (isnan(offsetValue)) {
    delay(500);
    scale.tare(20);  // Tare the scale with 20 readings
    delay(500);
    printMessageLn("Scale tared!");
  } else {
    printMessageLn("Offset Value set From Memory: " + String(offsetValue));
    scale.set_offset(offsetValue);
  }
}
