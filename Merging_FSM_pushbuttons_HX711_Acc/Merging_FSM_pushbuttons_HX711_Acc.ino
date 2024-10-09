String file_name = "Programa: Merging_FSM_pushbuttons_HX711_Acc.ino"; 

#include <HX711.h>
#include "BluetoothSerial.h"  // Include the BluetoothSerial library
#include "esp_system.h"       // Include the ESP32 system header
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

//-------------------- Bluetooth ------------------------
#define BT_PRINT        // Print over Bluetooth
#define BETTER_PLOTTER  // Print over Serial Port

#ifdef BT_PRINT
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error ERROR 101: Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif

  #if !defined(CONFIG_BT_SPP_ENABLED)
    #error ERROR 102: Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
  #endif

  BluetoothSerial SerialBT;  // Create an instance of BluetoothSerial called "SerialBT"
#endif

//-------------------- Accelerometer------------------------
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "NXP_FXOS_FXAS.h" //         THIS LIBRARY HAS TO BE INCLUDE AFTER DECLARATING THE Adafruit Sensors

#define FILTER_UPDATE_RATE_HZ 50
#define PRINT_EVERY_N_UPDATES 10
uint32_t timestamp;

// -----------   pick a filter! slower == better quality output  ------------
//Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;  
#endif

//-------------------- FSM ans Digital I/0------------------------
const int keyPadPins[] = { 4, 0, 2, 15 };
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;
unsigned long lastToggleTime = 0; // Keep track of the last time the LED state was toggled

//-------------------- HX711 ------------------------
HX711 scale;
uint8_t dataPin = 34;
uint8_t clockPin = 23;
uint8_t readingsNo = 1; // Number of readings per meassurement
const int calibrationAddress = 321;

// Define the states
enum State { STATE_1,   //default state, always do STATE_1
             STATE_2,
             STATE_3,
             STATE_4 };
State state = STATE_1;

// Define the blink intervals for each state (in milliseconds)
//const int blinkIntervals[] = {500, 500, 250, 125};

void setup() {
  //-------------------- SERIAL ------------------------
  #ifdef BETTER_PLOTTER 
    Serial.begin(115200);
    if (Serial.available()) {}
    Serial.println(file_name);
  #endif
  //-------------------- Digital I/0------------------------
  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  pinMode(ledPin, OUTPUT);

  //-------------------- Bluetooth ------------------------
  #ifdef BT_PRINT
    uint64_t chipid = 102390; //some initial number
    chipid = ESP.getEfuseMac();
    String chipIdString = String((uint32_t)chipid).substring(7);// Get the MAC address of the ESP32
    String deviceName = "Crutch-BT-" + chipIdString;  // Create a unique device name
    SerialBT.begin(deviceName);                                   // Start the Bluetooth with the unique device name
  #endif

#ifdef USE_PIN
  SerialBT.setPin(pin);
 printMessageLn("Using PIN");
#endif

//-------------------- Accelerometer------------------------
if (!cal.begin()) {
    printMessageLn("ERROR 201: Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    printMessageLn("No calibration loaded/found");
  } else {
    printCalibration();
    delay(3000);
  }

  if (!init_sensors()) {
   printMessageLn("ERROR 202: Failed to find sensors");
    while (1) delay(10);
  }

  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
 
//-------------------- HX711 ------------------------
  scale.begin(dataPin, clockPin); 
  EEPROM.begin(512);// Initialize EEPROM with a size of 512 bytes
  // Load calibration from EEPROM
  loadFromMemory();

  // if (chipIdString.equals("036")){ //123380739632932) {
  //   scale.set_scale(-5.752855); // Set the scale
  //   scale.set_offset(-1314026); // Set the offset
  //   printMessageLn("Crutch 1");
  // } else if (chipIdString.equals("796")){
  //   scale.set_offset(-1314026);  //Crutch 2
  //   scale.set_scale(-5.639015);    //Crutch 2
  //   printMessageLn("Crutch 2");
  // } else {
  //   printMessageLn("Error 301: ID number don't match, scale not calibrated");
  //   printMessageLn(chipIdString);
  // }

  printMessage("------------ HX711--------------");
  scale.set_average_mode(); //set_medavg_mode();   //set mode
  printMessage("MODE: ");
  printMessageLn(getModeNameHX711(scale.get_mode()));
  printMessage("GAIN: ");
  printMessageLn(getGainNameHX711(scale.get_gain()));

}

void loop() {
  static uint8_t counter = 0; // used for the 
  bool read_flag = false;
  bool print_flag = false;
  float gx, gy, gz;

  //------------------- FSM -----------------------------
  for (int i = 0; i < numButtons; i++) {  // Check each button 
    if (digitalRead(keyPadPins[i]) == LOW) {
      // If the button is pressed, change the state
      state = static_cast<State>(i);
      break;
    }
  }

  switch (state) {
    case STATE_1:
      // default state always do
      break;
    case STATE_2:
      // Increase number of readins per measurement
      readingsNo++;
      delay(300);
      
      state = STATE_1;
      break;
    case STATE_3:
      // reset number of readingsNo
      readingsNo = 1;

      state = STATE_1;
      break;
    case STATE_4:
      printScaleParams();
      delay(500);
      scale.tare(20);// tare the scale
      delay(500);

      state = STATE_1;
      break;
  }

  if ((millis() - timestamp) >= (1000 / FILTER_UPDATE_RATE_HZ)) {
    read_flag = true;
    timestamp = millis();
  }

  if (++counter >= PRINT_EVERY_N_UPDATES) {
    print_flag = true;
    digitalWrite(ledPin, !digitalRead(ledPin)); // LED toggle
  }

if(read_flag){
    read_flag = false;

    // Read the motion sensors
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    #if defined(AHRS_DEBUG_OUTPUT)
      printMessage("I2C took "); printMessage(String(millis()-timestamp));  printMessageLn(" ms");
    #endif

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, 
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    #if defined(AHRS_DEBUG_OUTPUT)
      printMessage("Update took "); 
      printMessage(String(millis()-timestamp));
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
      printMessage(String(mag.magnetic.z, 4));printMessageLn("");
    #endif
}

if(print_flag){
  print_flag = false;

  printForce_Orientation(filter);

  printMessage("\n");  //printQuaternion(filter); //<<<<<<<<<<<<<<<<<<--------------------------------

  #if defined(AHRS_DEBUG_OUTPUT)
    printMessage("Took "); printMessage(String(millis()-timestamp,4));printMessageLn(" ms");
  #endif  
  }

  #if defined(BETTER_PLOTTER) && defined(BT_PRINT)
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
  #endif

  delay(1);
}


void printMessage(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.print(message);  // Print over Bluetooth
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.print(message);  // Print over Serial Port
  #endif
}

void printMessageLn(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.println(message);  // Print over Bluetooth
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.println(message);  // Print over Serial Port
  #endif
}

void printCalibration(){
 printMessageLn("Initialization or Calibration loaded/found successfully");
 printMessageLn("Calibrations found: ");
  printMessage("\tMagnetic Hard Offset: ");
  for (int i=0; i<3; i++) {
    printMessage(String(cal.mag_hardiron[i])); 
    if (i != 2) printMessage(", ");
  }
 printMessageLn("");
  printMessage("\tMagnetic Soft Offset: ");
  for (int i=0; i<9; i++) {
    printMessage(String(cal.mag_softiron[i])); 
    if (i != 8) printMessage(", ");
  }
 printMessageLn("");
 printMessage("\tMagnetic Field Magnitude: ");
 printMessageLn(String(cal.mag_field));
  printMessage("\tGyro Zero Rate Offset: ");
  for (int i=0; i<3; i++) {
    printMessage(String(cal.gyro_zerorate[i])); 
    if (i != 2) printMessage(", ");
  }
 printMessageLn("");
  printMessage("\tAccel Zero G Offset: ");
  for (int i=0; i<3; i++) {
    printMessage(String(cal.accel_zerog[i])); 
    if (i != 2) printMessage(", ");
  }
 printMessageLn("");
}

void printScaleParams(){
  // float t = scale.get_tare();
  long of = scale.get_offset();
  // float sc = scale.get_scale();
  // uint8_t g = scale.get_gain();
  // float v = scale.get_value();
  // float avg = scale.read_average(5);
  // uint32_t l = scale.read();
  printMessage(String(0));
  printMessage("\t");
  printMessage(String(100 * readingsNo));
  printMessage("\t");
  printMessage(String(100 * state));

  printMessage("\t");
  printMessage(String(scale.get_mode()));
  printMessage(" - offset: ");
  printMessage(String(of));
  // printMessage("  uint32_t: ");
  // printMessageLn((uint32_t)of);
}

void printForce_Orientation(Adafruit_Madgwick filter){
    //------------------- AHRS -----------------------------
  float roll, pitch, heading;
  float f;

  if (scale.is_ready()) {
    f = scale.get_units(readingsNo);    // Get the current Measurement from the HX711  
  }else{
    printMessageLn("Error 302: Scale is not ready");
  }
  //------------------- AHRS -----------------------------
  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  printMessage(String(millis()));
  printMessage("\t");
  printMessage("F: ");
  printMessage(String(f));
  printMessage(", ");
  printMessage("Ori: ");
  printMessage(String(heading,4));
  printMessage(", ");
  printMessage(String(pitch,4));
  printMessage(", ");
  printMessage(String(roll,4));
  printMessage("\t"); 
}

void printQuaternion(Adafruit_Madgwick filter){
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  printMessage("Quaternion: ");
  printMessage(String(qw,4));
  printMessage(", ");
  printMessage(String(qx,4));
  printMessage(", ");
  printMessage(String(qy,4));
  printMessage(", ");
  printMessage(String(qz,4));
  printMessageLn(",");
}

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
