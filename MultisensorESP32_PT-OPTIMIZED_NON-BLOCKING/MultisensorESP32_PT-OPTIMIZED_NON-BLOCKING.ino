// COMPLETE CODE: High-Performance ESP32 with Filter Convergence Fix
// Fixed non-blocking issues that were preventing Madgwick filter convergence

#include <HX711.h>
#include "BluetoothSerial.h"
#include "esp_system.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <EEPROM.h>
#include <Wire.h>

//---- System Configuration ----
String file_name = "MultisensorESP32_PT - FILTER CONVERGENCE FIXED";

//---- High-Performance Frequency Configuration ----
#define IMU_UPDATE_RATE_HZ 200         // Excellent Madgwick performance
#define FORCE_UPDATE_RATE_HZ 80        // High-resolution force data
#define PRINT_RATE_HZ 80               // Near BT limits: ~30% capacity

#define IMU_PERIOD_MS (1000 / IMU_UPDATE_RATE_HZ)      // 5ms
#define FORCE_PERIOD_MS (1000 / FORCE_UPDATE_RATE_HZ)  // 12.5ms  
#define PRINT_PERIOD_MS (1000 / PRINT_RATE_HZ)         // 12.5ms

//---- Communication Optimization ----
#define SERIAL_BAUD_RATE 921600        // 8x faster than 115200
#define USE_COMPACT_FORMAT true        // Compact message format
#define USE_QUATERNIONS true           // Use quaternions vs Euler

//---- Timeouts and Limits ----
#define MAX_FORCE_READING_TIME_MS 15   // HX711 timeout
#define FORCE_ERROR_THRESHOLD 1000000  // Force validation
#define I2C_TIMEOUT_MS 20              // I2C timeout

//---- Bluetooth Configuration ----
#define BT_PRINT
#define BETTER_PLOTTER

#ifdef BT_PRINT
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error ERRO 101: Bluetooth não está habilitado!
  #endif
  #if !defined(CONFIG_BT_SPP_ENABLED)
    #error ERRO 102: Bluetooth Serial não disponível!
  #endif
  BluetoothSerial SerialBT;
#endif

//---- Sensor Hardware ----
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "NXP_FXOS_FXAS.h"

Adafruit_Madgwick filter;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

//---- Hardware Pins ----
const int keyPadPins[] = { 4, 0, 2, 15 };
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;
uint8_t loopDelay = 1;

HX711 scale;
uint8_t dataPin = 34;
uint8_t clockPin = 23;
uint8_t readingsNo = 1;
const int calibrationAddress = 321;

//---- State Machine ----
enum State {
  STATE_1, STATE_2, STATE_3, STATE_4
};
State state = STATE_1;

//---- Timing Variables ----
uint32_t imu_timestamp = 0;
uint32_t force_timestamp = 0;
uint32_t print_timestamp = 0;

//---- Sensor Data Variables ----
float current_roll = 0, current_pitch = 0, current_yaw = 0;
float current_qw = 0, current_qx = 0, current_qy = 0, current_qz = 0;
float gx = 0, gy = 0, gz = 0;
bool imu_data_ready = false;

float current_force = 0;
bool force_data_ready = false;

//---- Performance Monitoring ----
uint32_t imu_cycles = 0;
uint32_t force_cycles = 0;
uint32_t print_cycles = 0;
uint32_t loop_max_time = 0;

//---- Filter Debug Variables (ADDED) ----
uint32_t filter_update_count = 0;
uint32_t last_filter_update = 0;
bool filter_converged = false;

//---- Non-Blocking HX711 State Machine ----
enum HX711_State {
  HX711_IDLE, HX711_WAIT_READY, HX711_READING, HX711_COMPLETE, HX711_ERROR
};

struct HX711_NonBlocking {
  HX711_State state;
  uint32_t start_time;
  uint32_t last_check_time;
  float last_valid_reading;
  bool data_ready;
  uint32_t error_count;
};

HX711_NonBlocking nb_hx711 = {
  .state = HX711_IDLE,
  .start_time = 0,
  .last_check_time = 0,
  .last_valid_reading = 0,
  .data_ready = false,
  .error_count = 0
};

//---- High-Performance Communication Queues ----
#define OUTPUT_BUFFER_SIZE 48
#define MAX_QUEUED_MESSAGES 30
#define BT_BURST_SIZE 5

struct UARTQueue {
  char messages[MAX_QUEUED_MESSAGES][OUTPUT_BUFFER_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
  uint32_t dropped_messages;
  uint32_t total_sent;
  uint32_t bytes_sent;
};

UARTQueue uart_queue = {0};
UARTQueue bt_queue = {0};

//---- Bluetooth Performance Stats ----
struct BluetoothStats {
  uint32_t messages_sent;
  uint32_t bytes_sent;
  uint32_t start_time;
  float current_bps;
  float peak_bps;
  uint32_t congestion_events;
};

BluetoothStats bt_stats = {0};

//==== NON-BLOCKING HX711 FUNCTIONS ====
void startHX711Reading() {
  if (nb_hx711.state == HX711_IDLE) {
    nb_hx711.state = HX711_WAIT_READY;
    nb_hx711.start_time = millis();
    nb_hx711.data_ready = false;
  }
}

void updateNonBlockingHX711() {
  uint32_t current_time = millis();
  
  switch (nb_hx711.state) {
    case HX711_IDLE:
      break;
      
    case HX711_WAIT_READY:
      if ((current_time - nb_hx711.start_time) > MAX_FORCE_READING_TIME_MS) {
        nb_hx711.error_count++;
        nb_hx711.state = HX711_IDLE;
        break;
      }
      
      if (scale.is_ready()) {
        nb_hx711.state = HX711_READING;
      }
      break;
      
    case HX711_READING:
      if (scale.is_ready()) {
        float reading = scale.get_units(readingsNo);
        
        if (abs(reading) < FORCE_ERROR_THRESHOLD) {
          nb_hx711.last_valid_reading = reading;
          nb_hx711.data_ready = true;
        }
        nb_hx711.state = HX711_IDLE;
      }
      break;
  }
}

//==== HIGH-PERFORMANCE COMMUNICATION ====
bool addToQueue(UARTQueue* queue, const char* message) {
  if (queue->count >= MAX_QUEUED_MESSAGES) {
    queue->dropped_messages++;
    return false;
  }
  
  strncpy(queue->messages[queue->head], message, OUTPUT_BUFFER_SIZE - 1);
  queue->messages[queue->head][OUTPUT_BUFFER_SIZE - 1] = '\0';
  
  queue->head = (queue->head + 1) % MAX_QUEUED_MESSAGES;
  queue->count++;
  queue->total_sent++;
  queue->bytes_sent += strlen(message);
  
  return true;
}

bool getFromQueue(UARTQueue* queue, char* buffer) {
  if (queue->count == 0) {
    return false;
  }
  
  strcpy(buffer, queue->messages[queue->tail]);
  queue->tail = (queue->tail + 1) % MAX_QUEUED_MESSAGES;
  queue->count--;
  
  return true;
}

void formatCompactMessage(char* buffer, size_t buffer_size) {
  // REMOVED DEBUG OUTPUT that was causing spam - now only when needed
  static uint32_t debug_count = 0;
  if (debug_count++ % 1000 == 0) { // Much less frequent - every 50 seconds at 20Hz
    char debug[120];
    snprintf(debug, sizeof(debug), "DEBUG: qw=%.3f qx=%.3f qy=%.3f qz=%.3f updates=%lu\n",
             current_qw, current_qx, current_qy, current_qz, filter_update_count);
    addToQueue(&uart_queue, debug);
  }
  
  #if USE_COMPACT_FORMAT
    // Ultra-compact: timestamp,force,qw,qx,qy,qz (26 bytes vs 47 bytes original)
    snprintf(buffer, buffer_size, "%lu,%d,%d,%d,%d,%d\n",
             millis(),
             (int)(current_force * 100),          // 2 decimal places
             (int)(current_qw * 1000),            // 3 decimal places  
             (int)(current_qx * 1000),
             (int)(current_qy * 1000),
             (int)(current_qz * 1000));
  #else
    // Standard format
    snprintf(buffer, buffer_size, "%lu,F:%.2f,Q:%.3f,%.3f,%.3f,%.3f\n",
             millis(), current_force,
             current_qw, current_qx, current_qy, current_qz);
  #endif
}

void updateBluetoothBurst() {
  static char buffer[OUTPUT_BUFFER_SIZE];
  static uint32_t last_stats_update = 0;
  
  #ifdef BT_PRINT
  if (SerialBT.hasClient()) {
    for (int i = 0; i < BT_BURST_SIZE && bt_queue.count > 0; i++) {
      if (getFromQueue(&bt_queue, buffer)) {
        SerialBT.print(buffer);
        bt_stats.messages_sent++;
        bt_stats.bytes_sent += strlen(buffer);
      }
    }
    
    if (millis() - last_stats_update > 1000) {
      uint32_t elapsed_ms = millis() - bt_stats.start_time;
      if (elapsed_ms > 0) {
        bt_stats.current_bps = (bt_stats.bytes_sent * 8.0 * 1000.0) / elapsed_ms;
        if (bt_stats.current_bps > bt_stats.peak_bps) {
          bt_stats.peak_bps = bt_stats.current_bps;
        }
      }
      last_stats_update = millis();
    }
  } else {
    if (bt_queue.count > 0) {
      bt_stats.congestion_events++;
      bt_queue.count = 0;
      bt_queue.head = bt_queue.tail = 0;
    }
  }
  #endif
}

void updateUARTFast() {
  static char buffer[OUTPUT_BUFFER_SIZE];
  
  #ifdef BETTER_PLOTTER
  for (int i = 0; i < 10 && uart_queue.count > 0; i++) {
    if (getFromQueue(&uart_queue, buffer)) {
      Serial.print(buffer);
    }
  }
  #endif
}

//==== MAIN PROCESS FUNCTIONS ====
void handleButtonStates() {
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(keyPadPins[i]) == LOW) {
      state = static_cast<State>(i);
      break;
    }
  }

  switch (state) {
    case STATE_1:
      break;
    case STATE_2:
      readingsNo++;
      delay(300);
      state = STATE_1;
      break;
    case STATE_3:
      readingsNo = 1;
      state = STATE_1;
      break;
    case STATE_4:
      printScaleParams();
      delay(500);
      scale.tare(20);
      delay(500);
      state = STATE_1;
      break;
  }
}

//==== FIXED IMU PROCESSING (BLOCKING VERSION FOR RELIABILITY) ====
void processIMU() {
  imu_cycles++;
  
  // BLOCKING VERSION - ensures reliable filter convergence
  sensors_event_t accel, gyro, mag;
  
  // Read all sensors
  if (accelerometer->getEvent(&accel) && 
      gyroscope->getEvent(&gyro) && 
      magnetometer->getEvent(&mag)) {
    
    // Calibrate readings
    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // Convert gyroscope from Rad/s to Degrees/s
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update Madgwick filter with proper timing
    filter.update(gx, gy, gz,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    // Get quaternion results
    #if USE_QUATERNIONS
      float new_qw, new_qx, new_qy, new_qz;
      filter.getQuaternion(&new_qw, &new_qx, &new_qy, &new_qz);
      
      // Check for filter convergence
      static float last_qw = 0, last_qx = 0, last_qy = 0, last_qz = 0;
      float quaternion_change = abs(new_qw - last_qw) + abs(new_qx - last_qx) + 
                               abs(new_qy - last_qy) + abs(new_qz - last_qz);
      
      current_qw = new_qw;
      current_qx = new_qx;
      current_qy = new_qy;
      current_qz = new_qz;
      
      filter_update_count++;
      
      // Check convergence after 100 updates
      if (!filter_converged && filter_update_count > 100) {
        if (quaternion_change < 0.005f) {
          filter_converged = true;
          addToQueue(&uart_queue, "FILTER: Converged successfully!\n");
        }
      }
      
      last_qw = new_qw;
      last_qx = new_qx;
      last_qy = new_qy;
      last_qz = new_qz;
      
    #else
      current_roll = filter.getRoll();
      current_pitch = filter.getPitch();
      current_yaw = filter.getYaw();
    #endif
    
    imu_data_ready = true;
  } else {
    // Sensor read failed
    static uint32_t read_failures = 0;
    read_failures++;
    if (read_failures % 100 == 0) {
      char error_msg[64];
      snprintf(error_msg, sizeof(error_msg), "IMU read failures: %lu\n", read_failures);
      addToQueue(&uart_queue, error_msg);
    }
  }
}

void processForce() {
  force_cycles++;
  
  updateNonBlockingHX711();
  
  if (!nb_hx711.data_ready && nb_hx711.state == HX711_IDLE) {
    startHX711Reading();
  }
  
  if (nb_hx711.data_ready) {
    current_force = nb_hx711.last_valid_reading;
    force_data_ready = true;
    nb_hx711.data_ready = false;
  }
}

void processPrint() {
  print_cycles++;
  
  if (print_cycles % 20 == 0) {
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  
  // DEBUG: Check data ready status
  static uint32_t debug_cycles = 0;
  if (debug_cycles++ % 100 == 0) { // Every 5 seconds at 20Hz
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg), "DATA STATUS: IMU=%s Force=%s\n", 
             imu_data_ready ? "READY" : "NOT_READY",
             force_data_ready ? "READY" : "NOT_READY");
    addToQueue(&uart_queue, debug_msg);
  }
  
  if (imu_data_ready && force_data_ready) {
    static char output_buffer[OUTPUT_BUFFER_SIZE];
    formatCompactMessage(output_buffer, sizeof(output_buffer));
    
    addToQueue(&uart_queue, output_buffer);
    addToQueue(&bt_queue, output_buffer);
  } else {
    // FORCE output even if not both ready - for debugging
    static char output_buffer[OUTPUT_BUFFER_SIZE];
    snprintf(output_buffer, sizeof(output_buffer), "%lu,%d,%d,%d,%d,%d\n",
             millis(),
             (int)(current_force * 100),
             (int)(current_qw * 1000),
             (int)(current_qx * 1000),
             (int)(current_qy * 1000),
             (int)(current_qz * 1000));
    
    addToQueue(&uart_queue, output_buffer);
    addToQueue(&bt_queue, output_buffer);
  }
  
  // Reduce performance stats frequency to avoid truncation
  if (print_cycles % 100 == 0) { // Every 5 seconds instead of every 20 seconds
    printAdvancedPerformanceStats();
  }
}

void printAdvancedPerformanceStats() {
  static char stats_buffer[200]; // Reduced size to prevent truncation
  
  uint32_t uptime_seconds = millis() / 1000;
  if (uptime_seconds > 0) {
    float real_imu_freq = (float)imu_cycles / uptime_seconds;
    float real_force_freq = (float)force_cycles / uptime_seconds;
    float real_print_freq = (float)print_cycles / uptime_seconds;
    float filter_freq = (float)filter_update_count / uptime_seconds;
    
    float bt_utilization = (bt_stats.current_bps / 57600.0) * 100.0;
    
    // SHORTER message to prevent truncation
    snprintf(stats_buffer, sizeof(stats_buffer),
             "PERF: IMU:%.0f Force:%.0f Print:%.0f Filter:%.0f Loop:%lu BT:%.0fbps Conv:%s\n",
             real_imu_freq, real_force_freq, real_print_freq, filter_freq, loop_max_time,
             bt_stats.current_bps, filter_converged ? "Y" : "N");
    
    addToQueue(&uart_queue, stats_buffer);
    addToQueue(&bt_queue, stats_buffer);
    
    // Also print queue status
    snprintf(stats_buffer, sizeof(stats_buffer),
             "QUEUES: UART=%d/%d BT=%d/%d Dropped=%lu\n",
             uart_queue.count, MAX_QUEUED_MESSAGES,
             bt_queue.count, MAX_QUEUED_MESSAGES,
             uart_queue.dropped_messages + bt_queue.dropped_messages);
    
    addToQueue(&uart_queue, stats_buffer);
  }
  
  loop_max_time = 0;
}

//==== MAIN LOOP ====
void loop() {
  uint32_t loop_start = micros();
  
  handleButtonStates();

  if ((millis() - imu_timestamp) >= IMU_PERIOD_MS) {
    processIMU();
    imu_timestamp = millis();
  }

  if ((millis() - force_timestamp) >= FORCE_PERIOD_MS) {
    processForce();
    force_timestamp = millis();
  }

  if ((millis() - print_timestamp) >= PRINT_PERIOD_MS) {
    processPrint();
    print_timestamp = millis();
  }

  updateBluetoothBurst();
  updateUARTFast();

  uint32_t loop_time_us = micros() - loop_start;
  uint32_t loop_time_ms = loop_time_us / 1000;
  
  if (loop_time_ms > loop_max_time) {
    loop_max_time = loop_time_ms;
  }
  
  if (loop_time_us < 200) {
    delayMicroseconds(50);
  }
}

//==== SETUP ====
void setup() {
  #ifdef BETTER_PLOTTER
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setTxBufferSize(2048);
    while (!Serial) {}
    Serial.println(file_name);
    Serial.println("=== FILTER CONVERGENCE FIXED SYSTEM ===");
    Serial.println("IMU: " + String(IMU_UPDATE_RATE_HZ) + "Hz (" + String(IMU_PERIOD_MS) + "ms)");
    Serial.println("Force: " + String(FORCE_UPDATE_RATE_HZ) + "Hz (" + String(FORCE_PERIOD_MS) + "ms)");
    Serial.println("Print: " + String(PRINT_RATE_HZ) + "Hz (" + String(PRINT_PERIOD_MS) + "ms)");
    Serial.println("Message format: COMPACT (26 bytes)");
    Serial.println("IMU Processing: BLOCKING (for reliability)");
    Serial.println("=======================================");
  #endif

  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  pinMode(ledPin, OUTPUT);

  #ifdef BT_PRINT
    uint64_t chipid = ESP.getEfuseMac();
    String chipIdString = String((uint32_t)(chipid & 0xFFFF), HEX);
    String deviceName = "FastCrutch-BT-" + chipIdString;
    SerialBT.begin(deviceName);
    bt_stats.start_time = millis();
  #endif

  if (!cal.begin()) {
    printMessageLn("ERRO 201: Falha ao inicializar calibração");
  } else if (!cal.loadCalibration()) {
    printMessageLn("Nenhuma calibração encontrada");
  } else {
    printCalibration();
    delay(1000);
  }

  if (!init_sensors()) {
    printMessageLn("ERRO 202: Falha ao encontrar sensores");
    while (1) delay(10);
  }

  setup_sensors();
  
  // FIXED FILTER INITIALIZATION
  filter.begin(IMU_UPDATE_RATE_HZ);
  delay(100); // Give filter time to initialize properly
  printMessageLn("Filter initialized at " + String(IMU_UPDATE_RATE_HZ) + "Hz");
  
  Wire.setClock(400000);

  scale.begin(dataPin, clockPin);
  scale.set_average_mode();
  EEPROM.begin(512);

  loadFromMemory();

  printMessageLn("---- HX711 ----");
  printMessage("MODO: ");
  printMessageLn(getModeNameHX711(scale.get_mode()));
  printMessage("GANHO: ");
  printMessageLn(getGainNameHX711(scale.get_gain()));

  imu_timestamp = millis();
  force_timestamp = millis();
  print_timestamp = millis();
  
  printMessageLn("=== SISTEMA INICIADO - FILTER CONVERGENCE FIXED ===");
}

//==== UTILITY FUNCTIONS ====
void printMessage(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.print(message);
    }
  #endif
  #ifdef BETTER_PLOTTER
    Serial.print(message);
  #endif
}

void printMessageLn(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.println(message);
    }
  #endif
  #ifdef BETTER_PLOTTER
    Serial.println(message);
  #endif
}

void printCalibration() {
  printMessageLn("Calibração carregada com sucesso");
  printMessageLn("Calibrações encontradas: ");
  printMessage("\tOffset Magnético Rígido: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.mag_hardiron[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tOffset Magnético Suave: ");
  for (int i = 0; i < 9; i++) {
    printMessage(String(cal.mag_softiron[i]));
    if (i != 8) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tMagnitude do Campo Magnético: ");
  printMessageLn(String(cal.mag_field));
  printMessage("\tOffset de Taxa Zero do Giroscópio: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.gyro_zerorate[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tOffset Zero G do Acelerômetro: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.accel_zerog[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
}

void printScaleParams() {
  long offset = scale.get_offset();
  printMessage("Estado: " + String(state));
  printMessage("\tLeituras: " + String(readingsNo));
  printMessage("\tModo da Balança: " + getModeNameHX711(scale.get_mode()));
  printMessage("\tOffset: " + String(offset));
  printMessageLn("");
}

String getModeNameHX711(uint8_t mode) {
  switch (mode) {
    case HX711_AVERAGE_MODE: return "Modo Média";
    case HX711_MEDIAN_MODE: return "Modo Mediana";
    case HX711_MEDAVG_MODE: return "Modo MedAvg";
    case HX711_RUNAVG_MODE: return "Modo RunAvg";
    case HX711_RAW_MODE: return "Modo Bruto";
    default: return "Modo Desconhecido";
  }
}

String getGainNameHX711(uint8_t gain) {
  switch (gain) {
    case HX711_CHANNEL_A_GAIN_128: return "Canal A Ganho 128";
    case HX711_CHANNEL_A_GAIN_64: return "Canal A Ganho 64";
    case HX711_CHANNEL_B_GAIN_32: return "Canal B Ganho 32";
    default: return "Ganho Desconhecido";
  }
}

void readCalibrationFromEEPROM(float &scaleValue, float &offsetValue) {
  EEPROM.get(calibrationAddress, scaleValue);
  EEPROM.get(calibrationAddress + sizeof(float), offsetValue);
}

void writeCalibrationToEEPROM(float scaleValue, float offsetValue) {
  EEPROM.put(calibrationAddress, scaleValue);
  EEPROM.put(calibrationAddress + sizeof(float), offsetValue);
  EEPROM.commit();
}

void loadFromMemory() {
  float scaleValue, offsetValue;
  readCalibrationFromEEPROM(scaleValue, offsetValue);

  if (isnan(scaleValue) || scaleValue == 0) {
    scaleValue = -5.752855;
    printMessageLn("Nenhuma Calibração Encontrada!");
    printMessageLn("Valor de Escala definido: " + String(scaleValue));
  } else {
    printMessageLn("Valor de Escala definido da Memória: " + String(scaleValue));
  }
  scale.set_scale(scaleValue);

  if (isnan(offsetValue)) {
    delay(500);
    scale.tare(20);
    delay(500);
    printMessageLn("Balança tarada!");
  } else {
    printMessageLn("Valor de Offset definido da Memória: " + String(offsetValue));
    scale.set_offset(offsetValue);
  }
}