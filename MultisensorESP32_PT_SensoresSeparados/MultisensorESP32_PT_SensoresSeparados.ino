// InclusÃo das bibliotecas necessÃrias
#include <HX711.h>    // Biblioteca para o amplificador de cÃ©lula de carga HX711
#include "BluetoothSerial.h"    // Inclui a biblioteca BluetoothSerial
#include "esp_system.h"    // Inclui o cabeÃ§alho do sistema ESP32
#include <Adafruit_Sensor_Calibration.h> // Para calibraÃ§Ão de sensores
#include <Adafruit_AHRS.h>    // Para sistemas AHRS (filtro Madgwick/Mahony)
#include <EEPROM.h>    // Inclui biblioteca EEPROM para salvar dados de calibraÃ§Ão
#include <Wire.h>    // Inclui biblioteca Wire para comunicaÃ§Ão I2C

//---- VariÃveis Globais e DefiniÃ§Ãµes ----

// Nome do arquivo do programa
String file_name = "Programa: MultisensorESP32_PT.ino - Sensores Separados OTIMIZADO";

//---- ConfiguraÃ§Ão de Frequências OTIMIZADA v2 ----
// Baseado nos testes: sistema pode mais, mas precisa evitar timeouts
#define IMU_UPDATE_RATE_HZ 80          // Aumentado: sistema aguenta mais
#define FORCE_UPDATE_RATE_HZ 30        // Aumentado: com timeout está seguro
#define PRINT_RATE_HZ 15               // Volta ao original

// Opções de saída otimizada
#define USE_QUATERNIONS true           // true = quaternions, false = euler
#define DECIMAL_PLACES 2               // Número de casas decimais para otimizar

// Cálculo dos períodos em ms
#define IMU_PERIOD_MS (1000 / IMU_UPDATE_RATE_HZ)      // 
#define FORCE_PERIOD_MS (1000 / FORCE_UPDATE_RATE_HZ)  // 
#define PRINT_PERIOD_MS (1000 / PRINT_RATE_HZ)         // 

#define MAX_FORCE_READING_TIME_MS 10   // Timeout para leitura do HX711
#define FORCE_ERROR_THRESHOLD 1000000  // Valor máximo aceitável para força

//---- Bluetooth ----
#define BT_PRINT    // Define para habilitar impressÃo Bluetooth
#define BETTER_PLOTTER  // Define para habilitar impressÃo na Porta Serial

// ConfiguraÃ§Ão do Bluetooth
#ifdef BT_PRINT
  // Garante que o Bluetooth esteja habilitado na configuraÃ§Ão
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error ERRO 101: Bluetooth nÃo estÃ habilitado! Por favor, execute `make menuconfig` e habilite-o
  #endif

  #if !defined(CONFIG_BT_SPP_ENABLED)
    #error ERRO 102: Bluetooth Serial nÃo disponÃ­vel ou nÃo habilitado. EstÃ disponÃ­vel apenas para o chip ESP32.
  #endif

  // Cria uma instÃ¢ncia de BluetoothSerial chamada "SerialBT"
  BluetoothSerial SerialBT;
#endif

//---- AcelerÃ´metro ----
// Declara ponteiros para sensores (deve ser antes de incluir NXP_FXOS_FXAS.h)
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Inclui a biblioteca para os sensores NXP (deve ser apÃ³s declarar os sensores Adafruit)
#include "NXP_FXOS_FXAS.h" // ESTA BIBLIOTECA DEVE SER INCLUÃDA APÃ"S DECLARAR OS Sensores Adafruit

// ConfiguraÃ§Ão do filtro AHRS
Adafruit_Madgwick filter;    // Filtro Madgwick

// Timestamps separados para cada processo
uint32_t imu_timestamp = 0;
uint32_t force_timestamp = 0;
uint32_t print_timestamp = 0;

// Armazenamento de calibraÃ§Ão do sensor (EEPROM ou cartÃo SD)
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

//---- FSM e E/S Digital ----
// Define os pinos para os botÃµes do teclado 
const int keyPadPins[] = { 4, 0, 2, 15 };
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;    // Pino para o LED
uint8_t  loopDelay = 1;  // Reduzido para melhor responsividade

//---- Amplificador de CÃ©lula de Carga HX711 ----
HX711 scale;    // Cria instÃ¢ncia do HX711
uint8_t dataPin = 34;    // Pino de dados para HX711
uint8_t clockPin = 23;    // Pino de clock para HX711
uint8_t readingsNo = 1;    // NÃºmero de leituras por mediÃ§Ão
const int calibrationAddress = 321;  // EndereÃ§o EEPROM para dados de calibraÃ§Ão

//---- DefiniÃ§Ãµes da MÃquina de Estados ----
// Define os estados
enum State {
  STATE_1,   // Estado padrÃo
  STATE_2,   // Aumenta o nÃºmero de leituras por mediÃ§Ão
  STATE_3,   // Reinicia o nÃºmero de leituras
  STATE_4    // Tara a balanÃ§a
};
State state = STATE_1;    // Inicializa o estado

//---- VariÃveis para dados dos sensores ----
// Dados IMU (atualizados a 80Hz)
float current_roll = 0, current_pitch = 0, current_yaw = 0;
float current_qw = 0, current_qx = 0, current_qy = 0, current_qz = 0; // Quaternions
float gx = 0, gy = 0, gz = 0;
bool imu_data_ready = false;

// Dados de força (atualizados a 30Hz)
float current_force = 0;
bool force_data_ready = false;

// Contadores para debug de performance
uint32_t imu_cycles = 0;
uint32_t force_cycles = 0; 
uint32_t print_cycles = 0;
uint32_t loop_max_time = 0;

// Variáveis para monitoramento de timeout do HX711
uint32_t force_read_start = 0;
bool force_reading_in_progress = false;

//---- FunÃ§Ãµes dos Processos Separados ----

void handleButtonStates() {
  // Verifica cada botÃo para pressionar
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(keyPadPins[i]) == LOW) {
      // Se o botÃo for pressionado, muda o estado
      state = static_cast<State>(i);
      break;
    }
  }

  // Trata as aÃ§Ãµes do estado
  switch (state) {
    case STATE_1:
      // Estado padrÃo, nada a fazer
      break;
    case STATE_2:
      // Aumenta o nÃºmero de leituras por mediÃ§Ão
      readingsNo++;
      delay(300);  // Previne mudanÃ§a rÃpida de estado
      state = STATE_1;  // Retorna ao estado padrÃo
      break;
    case STATE_3:
      // Reinicia o nÃºmero de leituras para 1
      readingsNo = 1;
      state = STATE_1;  // Retorna ao estado padrÃo
      break;
    case STATE_4:
      // Imprime parÃ¢metros da balanÃ§a e tara a balanÃ§a
      printScaleParams();
      delay(500);
      scale.tare(20);  // Tara a balanÃ§a com 20 leituras
      delay(500);
      state = STATE_1;  // Retorna ao estado padrÃo
      break;
  }
}

// Função otimizada para processo de força com timeout
void processForce() {
  force_cycles++;
  
  // Inicia leitura se não estiver em progresso
  if (!force_reading_in_progress) {
    if (scale.is_ready()) {
      force_reading_in_progress = true;
      force_read_start = millis();
    } else {
      // HX711 não está pronto, pula este ciclo
      return;
    }
  }
  
  // Verifica timeout na leitura
  if (force_reading_in_progress) {
    if ((millis() - force_read_start) > MAX_FORCE_READING_TIME_MS) {
      // Timeout! Cancela leitura
      force_reading_in_progress = false;
      printMessageLn("WARN: HX711 timeout!");
      return;
    }
    
    // Tenta fazer a leitura
    if (scale.is_ready()) {
      float raw_force = scale.get_units(readingsNo);
      
      // Validação do valor lido
      if (abs(raw_force) < FORCE_ERROR_THRESHOLD) {
        current_force = raw_force;
        force_data_ready = true;
      } else {
        printMessage("WARN: Force spike detected: ");
        printMessageLn(String(raw_force));
        // Mantém valor anterior
      }
      
      force_reading_in_progress = false;
    }
  }
}

// Função otimizada para processo IMU com verificação de tempo
void processIMU() {
  uint32_t imu_start = millis();
  imu_cycles++;
  
  // Lê os sensores de movimento
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  // Verifica se a leitura I2C demorou muito
  uint32_t i2c_time = millis() - imu_start;
  if (i2c_time > 5) {
    printMessage("WARN: I2C slow: ");
    printMessage(String(i2c_time));
    printMessageLn("ms");
  }

  // Calibra leituras do sensor
  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  // Giroscópio precisa ser convertido de Rad/s para Graus/s
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Atualiza o filtro SensorFusion
  filter.update(gx, gy, gz,
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // Obtém dados de orientação (quaternions são mais eficientes que euler)
  #if USE_QUATERNIONS
    filter.getQuaternion(&current_qw, &current_qx, &current_qy, &current_qz);
  #else
    current_roll = filter.getRoll();
    current_pitch = filter.getPitch();
    current_yaw = filter.getYaw();
  #endif
  
  imu_data_ready = true;
  
  // Monitor de tempo total do processo IMU
  uint32_t total_imu_time = millis() - imu_start;
  if (total_imu_time > 20) {
    printMessage("WARN: IMU process slow: ");
    printMessage(String(total_imu_time));
    printMessageLn("ms");
  }
}

// Função otimizada para impressão rápida usando buffer pré-alocado
void printSensorData() {
  // Buffer pré-alocado para construção rápida da string
  static char output_buffer[128];
  
  #if USE_QUATERNIONS
    // Formato quaternion otimizado: T F qW qX qY qZ
    snprintf(output_buffer, sizeof(output_buffer), 
             "%lu\tF:%.2f\tQ:%.3f,%.3f,%.3f,%.3f",
             millis(), 
             current_force,
             current_qw, current_qx, current_qy, current_qz);
  #else
    // Formato euler otimizado: T F Y P R
    snprintf(output_buffer, sizeof(output_buffer), 
             "%lu\tF:%.2f\tOr:Y:%.2f,P:%.2f,R:%.2f",
             millis(), 
             current_force,
             current_yaw, current_pitch, current_roll);
  #endif
  
  // Impressão única (muito mais rápida que múltiplas chamadas)
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.println(output_buffer);
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.println(output_buffer);
  #endif
}

// Função para processar print otimizada
void processPrint() {
  print_cycles++;
  
  // Alterna LED como indicador de atividade
  digitalWrite(ledPin, !digitalRead(ledPin));
  
  // Imprime dados apenas se tiver dados válidos
  if (imu_data_ready && force_data_ready) {
    printSensorData();
  }
  
  // A cada 50 prints (5 segundos com 10Hz), imprime estatísticas
  if (print_cycles % 50 == 0) {
    printPerformanceStats();
  }
}

void printPerformanceStats() {
  printMessageLn("=== PERFORMANCE STATS ===");
  printMessage("IMU cycles: "); printMessageLn(String(imu_cycles));
  printMessage("Force cycles: "); printMessageLn(String(force_cycles));
  printMessage("Print cycles: "); printMessageLn(String(print_cycles));
  printMessage("Max loop time: "); printMessage(String(loop_max_time)); printMessageLn("ms");
  
  // Calcula frequências reais
  uint32_t uptime_seconds = millis() / 1000;
  if (uptime_seconds > 0) {
    float real_imu_freq = (float)imu_cycles / uptime_seconds;
    float real_force_freq = (float)force_cycles / uptime_seconds;
    float real_print_freq = (float)print_cycles / uptime_seconds;
    
    printMessage("Real frequencies - IMU: "); printMessage(String(real_imu_freq, 1)); printMessage("Hz");
    printMessage(", Force: "); printMessage(String(real_force_freq, 1)); printMessage("Hz");
    printMessage(", Print: "); printMessage(String(real_print_freq, 1)); printMessageLn("Hz");
  }
  
  // Reset max loop time
  loop_max_time = 0;
  printMessageLn("========================");
}

//---- FunÃ§Ão de Loop Principal ----
void loop() {
  uint32_t loop_start = millis();
  bool any_process_ran = false;

  //---- MÃquina de Estados Finitos (FSM) - Sempre verifica ----
  handleButtonStates();

  //---- Processo IMU (80Hz) ----
  if ((millis() - imu_timestamp) >= IMU_PERIOD_MS) {
    processIMU();
    imu_timestamp = millis();
    any_process_ran = true;
  }

  //---- Processo Força (30Hz) ----  
  if ((millis() - force_timestamp) >= FORCE_PERIOD_MS) {
    processForce();
    force_timestamp = millis();
    any_process_ran = true;
  }

  //---- Processo Print (10Hz) ----
  if ((millis() - print_timestamp) >= PRINT_PERIOD_MS) {
    processPrint();
    print_timestamp = millis();
    any_process_ran = true;
  }

  // Monitoramento de performance
  uint32_t loop_time = millis() - loop_start;
  if (loop_time > loop_max_time) {
    loop_max_time = loop_time;
  }
  
  // Alerta se loop demorar muito
  if (loop_time > 10) {
    printMessage("CRITICAL: Loop overflow: ");
    printMessage(String(loop_time));
    printMessageLn("ms");
  }

  // Passa dados do Bluetooth para Serial se ambos estiverem definidos
  #if defined(BETTER_PLOTTER) && defined(BT_PRINT)
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
  #endif

  // Delay mínimo apenas se nenhum processo foi executado
  if (!any_process_ran) {
    delay(loopDelay);
  }
}

//---- FunÃ§Ão de ConfiguraÃ§Ão ----
void setup() {
  //---- ComunicaÃ§Ão Serial ----
  #ifdef BETTER_PLOTTER
    Serial.begin(115200);    // Inicia comunicaÃ§Ão Serial na taxa de 115200 baud
    while (!Serial) {}    // Aguarda a porta Serial estar disponÃ­vel
    Serial.println(file_name);    // Imprime o nome do arquivo do programa
    Serial.println("=== CONFIGURACAO DE FREQUENCIAS ===");
    Serial.println("IMU: " + String(IMU_UPDATE_RATE_HZ) + "Hz (" + String(IMU_PERIOD_MS) + "ms)");
    Serial.println("FORCE: " + String(FORCE_UPDATE_RATE_HZ) + "Hz (" + String(FORCE_PERIOD_MS) + "ms)");
    Serial.println("PRINT: " + String(PRINT_RATE_HZ) + "Hz (" + String(PRINT_PERIOD_MS) + "ms)");
    Serial.println("=====================================");
  #endif

  //---- Entrada/SaÃ­da Digital ----
  // Inicializa pinos do teclado como entrada com resistores pull-up
  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  // Inicializa pino do LED como saÃ­da
  pinMode(ledPin, OUTPUT);

  //---- InicializaÃ§Ão do Bluetooth ----
  #ifdef BT_PRINT
    // Gera um nome de dispositivo Ãºnico usando o endereÃ§o MAC do ESP32
    uint64_t chipid = ESP.getEfuseMac();
    String chipIdString = String((uint32_t)(chipid & 0xFFFF), HEX);
    String deviceName = "Calib-Crutch-BT-" + chipIdString;  // Cria um nome de dispositivo Ãºnico
    SerialBT.begin(deviceName);    // Inicia Bluetooth com o nome de dispositivo Ãºnico
  #endif

  //---- InicializaÃ§Ão do AcelerÃ´metro ----
  if (!cal.begin()) {
    printMessageLn("ERRO 201: Falha ao inicializar o auxiliar de calibraÃ§Ão");
  } else if (!cal.loadCalibration()) {
    printMessageLn("Nenhuma calibraÃ§Ão carregada/encontrada");
  } else {
    printCalibration();  // Imprime dados de calibraÃ§Ão
    delay(1000); // Reduzido delay
  }

  if (!init_sensors()) {
    printMessageLn("ERRO 202: Falha ao encontrar sensores");
    while (1) delay(10);  // Para se os sensores nÃo forem encontrados
  }

  setup_sensors();    // Configura sensores
  filter.begin(IMU_UPDATE_RATE_HZ);  // Inicializa o filtro com nova frequência
  
  Wire.setClock(100000);    // Reduzido de 400KHz para 100KHz para estabilidade

  //---- InicializaÃ§Ão do HX711 ----
  scale.begin(dataPin, clockPin);
  scale.set_average_mode();
  EEPROM.begin(512);    // Inicializa EEPROM com tamanho de 512 bytes

  // Reduzir número de leituras para acelerar
  if (readingsNo > 1) {
    readingsNo = 1;  // Força para uma leitura por medição
  }

  // Carrega calibraÃ§Ão da EEPROM
  loadFromMemory();

  printMessageLn("---- HX711 ----");
  printMessage("MODO: ");
  printMessageLn(getModeNameHX711(scale.get_mode()));
  printMessage("GANHO: ");
  printMessageLn(getGainNameHX711(scale.get_gain()));

  // Inicializa timestamps
  imu_timestamp = millis();
  force_timestamp = millis();
  print_timestamp = millis();
  
  printMessageLn("=== SISTEMA OTIMIZADO - QUATERNIONS + PRINT RAPIDO ===");
  printMessageLn("IMU: " + String(IMU_UPDATE_RATE_HZ) + "Hz");
  printMessageLn("Force: " + String(FORCE_UPDATE_RATE_HZ) + "Hz");
  printMessageLn("Print: " + String(PRINT_RATE_HZ) + "Hz");
  #if USE_QUATERNIONS
    printMessageLn("Orientacao: QUATERNIONS (qW,qX,qY,qZ)");
  #else
    printMessageLn("Orientacao: EULER (Yaw,Pitch,Roll)");
  #endif
}

//---- FunÃ§Ãµes Auxiliares (mantidas do código original) ----

// FunÃ§Ão para imprimir mensagens via Bluetooth e/ou Porta Serial
void printMessage(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.print(message);    // Imprime via Bluetooth
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.print(message);    // Imprime via Porta Serial
  #endif
}

// FunÃ§Ão para imprimir mensagens com nova linha via Bluetooth e/ou Porta Serial
void printMessageLn(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.println(message);    // Imprime via Bluetooth
    }
  #endif

  #ifdef BETTER_PLOTTER
    Serial.println(message);    // Imprime via Porta Serial
  #endif
}

// FunÃ§Ão para imprimir dados de calibraÃ§Ão
void printCalibration() {
  printMessageLn("InicializaÃ§Ão ou CalibraÃ§Ão carregada/encontrada com sucesso");
  printMessageLn("CalibraÃ§Ãµes encontradas: ");
  printMessage("\tOffset MagnÃ©tico RÃ­gido: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.mag_hardiron[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tOffset MagnÃ©tico Suave: ");
  for (int i = 0; i < 9; i++) {
    printMessage(String(cal.mag_softiron[i]));
    if (i != 8) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tMagnitude do Campo MagnÃ©tico: ");
  printMessageLn(String(cal.mag_field));
  printMessage("\tOffset de Taxa Zero do GiroscÃ³pio: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.gyro_zerorate[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tOffset Zero G do AcelerÃ´metro: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.accel_zerog[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
}

// FunÃ§Ão para imprimir parÃ¢metros da balanÃ§a
void printScaleParams() {
  long offset = scale.get_offset();
  printMessage("Estado: " + String(state));
  printMessage("\tLeituras: " + String(readingsNo));
  printMessage("\tModo da BalanÃ§a: " + getModeNameHX711(scale.get_mode()));
  printMessage("\tOffset: " + String(offset));
  printMessageLn("");
}

// FunÃ§Ão para obter o nome do modo do HX711
String getModeNameHX711(uint8_t mode) {
  switch (mode) {
    case HX711_AVERAGE_MODE:
      return "Modo MÃ©dia";
    case HX711_MEDIAN_MODE:
      return "Modo Mediana";
    case HX711_MEDAVG_MODE:
      return "Modo MedAvg";
    case HX711_RUNAVG_MODE:
      return "Modo RunAvg";
    case HX711_RAW_MODE:
      return "Modo Bruto";
    default:
      return "Modo Desconhecido";
  }
}

// FunÃ§Ão para obter o nome do ganho do HX711
String getGainNameHX711(uint8_t gain) {
  switch (gain) {
    case HX711_CHANNEL_A_GAIN_128:
      return "Canal A Ganho 128";
    case HX711_CHANNEL_A_GAIN_64:
      return "Canal A Ganho 64";
    case HX711_CHANNEL_B_GAIN_32:
      return "Canal B Ganho 32";
    default:
      return "Ganho Desconhecido";
  }
}

// FunÃ§Ão para ler dados de calibraÃ§Ão da EEPROM
void readCalibrationFromEEPROM(float &scaleValue, float &offsetValue) {
  EEPROM.get(calibrationAddress, scaleValue);
  EEPROM.get(calibrationAddress + sizeof(float), offsetValue);
}

// FunÃ§Ão para escrever dados de calibraÃ§Ão na EEPROM
void writeCalibrationToEEPROM(float scaleValue, float offsetValue) {
  EEPROM.put(calibrationAddress, scaleValue);
  EEPROM.put(calibrationAddress + sizeof(float), offsetValue);
  EEPROM.commit();  // Salva alteraÃ§Ãµes na EEPROM
}

// FunÃ§Ão para carregar dados de calibraÃ§Ão da memÃ³ria
void loadFromMemory() {
  float scaleValue, offsetValue;
  readCalibrationFromEEPROM(scaleValue, offsetValue);

  // Verifica se a EEPROM contÃ©m dados vÃlidos
  if (isnan(scaleValue) || scaleValue == 0) {
    scaleValue = -5.752855;  // Valor de escala padrÃo
    printMessageLn("Nenhuma CalibraÃ§Ão Encontrada!");
    printMessageLn("Valor de Escala definido: " + String(scaleValue));
  } else {
    printMessageLn("Valor de Escala definido da MemÃ³ria: " + String(scaleValue));
  }
  scale.set_scale(scaleValue);

  if (isnan(offsetValue)) {
    delay(500);
    scale.tare(20);  // Tara a balanÃ§a com 20 leituras
    delay(500);
    printMessageLn("BalanÃ§a tarada!");
  } else {
    printMessageLn("Valor de Offset definido da MemÃ³ria: " + String(offsetValue));
    scale.set_offset(offsetValue);
  }
}