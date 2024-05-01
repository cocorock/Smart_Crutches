// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

// -------------------------------------------------------------
#include "BluetoothSerial.h"
// #define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.
String device_name = "ESP32-BT-Crutch_R";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
// -------------------------------------------------------------
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
// #include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;  
#endif

#define FILTER_UPDATE_RATE_HZ 50
#define PRINT_EVERY_N_UPDATES 10
// #define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();
  
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  } else {
    Serial.println("Initialization or Calibration loaded/found successfully");
    Serial.println("Calibrations found: ");
    Serial.print("\tMagnetic Hard Offset: ");
    for (int i=0; i<3; i++) {
      Serial.print(cal.mag_hardiron[i]); 
      if (i != 2) Serial.print(", ");
    }
    Serial.println();
    
    Serial.print("\tMagnetic Soft Offset: ");
    for (int i=0; i<9; i++) {
      Serial.print(cal.mag_softiron[i]); 
      if (i != 8) Serial.print(", ");
    }
    Serial.println();

    Serial.print("\tMagnetic Field Magnitude: ");
    Serial.println(cal.mag_field);

    Serial.print("\tGyro Zero Rate Offset: ");
    for (int i=0; i<3; i++) {
      Serial.print(cal.gyro_zerorate[i]); 
      if (i != 2) Serial.print(", ");
    }
    Serial.println();

    Serial.print("\tAccel Zero G Offset: ");
    for (int i=0; i<3; i++) {
      Serial.print(cal.accel_zerog[i]); 
      if (i != 2) Serial.print(", ");
    }
    Serial.println();
    delay(3000);
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
}

float roll, pitch, heading;
float gx, gy, gz;
sensors_event_t accel, gyro, mag;
bool read_flag = false;

void loop() {
  if ((millis() - timestamp) >= (1000 / FILTER_UPDATE_RATE_HZ)) {
    read_flag = true;
    timestamp = millis();
  }else{
    // Serial.print("TimeStamp: ");
    // Serial.println(millis() - timestamp,4);
  }

  if(read_flag){
    read_flag = false;
     // Read the motion sensors
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    #if defined(AHRS_DEBUG_OUTPUT)
      Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
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
      Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
    #endif


    #if defined(AHRS_DEBUG_OUTPUT)
      Serial.print("Raw: ");
      Serial.print(accel.acceleration.x, 4); Serial.print(", ");
      Serial.print(accel.acceleration.y, 4); Serial.print(", ");
      Serial.print(accel.acceleration.z, 4); Serial.print(", ");
      Serial.print(gx, 4); Serial.print(", ");
      Serial.print(gy, 4); Serial.print(", ");
      Serial.print(gz, 4); Serial.print(", ");
      Serial.print(mag.magnetic.x, 4); Serial.print(", ");
      Serial.print(mag.magnetic.y, 4); Serial.print(", ");
      Serial.print(mag.magnetic.z, 4); Serial.println("");
    #endif

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    // Serial.print(millis());
    // Serial.print(' ');
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.println(roll);

    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    Serial.print("Quaternion: ");
    Serial.print(qw, 4);
    Serial.print(", ");
    Serial.print(qx, 4);
    Serial.print(", ");
    Serial.print(qy, 4);
    Serial.print(", ");
    Serial.println(qz, 4);  

    if (SerialBT.hasClient()){
      SerialBT.print("Orientation: ");
      SerialBT.print(heading);
      SerialBT.print(", ");
      SerialBT.print(pitch);
      SerialBT.print(", ");
      SerialBT.println(roll);
      
      SerialBT.print("Quaternion: ");
      SerialBT.print(qw, 4);
      SerialBT.print(", ");
      SerialBT.print(qx, 4);
      SerialBT.print(", ");
      SerialBT.print(qy, 4);
      SerialBT.print(", ");
      SerialBT.println(qz, 4);
    }
    
    #if defined(AHRS_DEBUG_OUTPUT)
      Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
    #endif  
  }
  
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
}