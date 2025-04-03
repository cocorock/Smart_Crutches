// Inclusão das bibliotecas necessárias
#include <HX711.h>    // Biblioteca para o amplificador de célula de carga HX711
#include "BluetoothSerial.h"    // Inclui a biblioteca BluetoothSerial
#include "esp_system.h"    // Inclui o cabeçalho do sistema ESP32
#include <Adafruit_Sensor_Calibration.h> // Para calibração de sensores
#include <Adafruit_AHRS.h>    // Para sistemas AHRS (filtro Madgwick/Mahony)
#include <EEPROM.h>    // Inclui biblioteca EEPROM para salvar dados de calibração
#include <Wire.h>    // Inclui biblioteca Wire para comunicação I2C

//---- Variáveis Globais e Definições ----

// Nome do arquivo do programa
String file_name = "Programa: Merging_FSM_ACC_BLE_HX_o1.ino";

//---- Bluetooth ----
#define BT_PRINT    // Define para habilitar impressão Bluetooth
#define BETTER_PLOTTER  // Define para habilitar impressão na Porta Serial

// Configuração do Bluetooth
#ifdef BT_PRINT
  // Garante que o Bluetooth esteja habilitado na configuração
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error ERRO 101: Bluetooth não está habilitado! Por favor, execute `make menuconfig` e habilite-o
  #endif

  #if !defined(CONFIG_BT_SPP_ENABLED)
    #error ERRO 102: Bluetooth Serial não disponível ou não habilitado. Está disponível apenas para o chip ESP32.
  #endif

  // Cria uma instância de BluetoothSerial chamada "SerialBT"
  BluetoothSerial SerialBT;
#endif

//---- Acelerômetro ----
// Declara ponteiros para sensores (deve ser antes de incluir NXP_FXOS_FXAS.h)
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Inclui a biblioteca para os sensores NXP (deve ser após declarar os sensores Adafruit)
#include "NXP_FXOS_FXAS.h" // ESTA BIBLIOTECA DEVE SER INCLUÍDA APÓS DECLARAR OS Sensores Adafruit

// Configuração do filtro AHRS
#define FILTER_UPDATE_RATE_HZ 50    // Taxa de atualização do filtro em Hz
#define PRINT_EVERY_N_UPDATES 10    // Imprime a cada n atualizações
uint32_t timestamp;    // Timestamp para atualizações do filtro

// Descomente o filtro que deseja usar (Madgwick é mais rápido que NXP, Mahony é o mais rápido/menor)
//Adafruit_NXPSensorFusion filter; // Mais lento
Adafruit_Madgwick filter;    // Mais rápido que NXP
//Adafruit_Mahony filter;    // Mais rápido/menor

// Armazenamento de calibração do sensor (EEPROM ou cartão SD)
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

//---- FSM e E/S Digital ----
// Define os pinos para os botões do teclado 
const int keyPadPins[] = { 4, 0, 2, 15 };
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;    // Pino para o LED
uint8_t  loopDelay = 2;

//---- Amplificador de Célula de Carga HX711 ----
HX711 scale;    // Cria instância do HX711
uint8_t dataPin = 34;    // Pino de dados para HX711
uint8_t clockPin = 23;    // Pino de clock para HX711
uint8_t readingsNo = 1;    // Número de leituras por medição
const int calibrationAddress = 321;  // Endereço EEPROM para dados de calibração

//---- Definições da Máquina de Estados ----
// Define os estados
enum State {
  STATE_1,   // Estado padrão
  STATE_2,   // Aumenta o número de leituras por medição
  STATE_3,   // Reinicia o número de leituras
  STATE_4    // Tara a balança
};
State state = STATE_1;    // Inicializa o estado

//---- Função de Configuração ----
void setup() {
  //---- Comunicação Serial ----
  #ifdef BETTER_PLOTTER
    Serial.begin(115200);    // Inicia comunicação Serial na taxa de 115200 baud
    while (!Serial) {}    // Aguarda a porta Serial estar disponível
    Serial.println(file_name);    // Imprime o nome do arquivo do programa
  #endif

  //---- Entrada/Saída Digital ----
  // Inicializa pinos do teclado como entrada com resistores pull-up
  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  // Inicializa pino do LED como saída
  pinMode(ledPin, OUTPUT);

  //---- Inicialização do Bluetooth ----
  #ifdef BT_PRINT
    // Gera um nome de dispositivo único usando o endereço MAC do ESP32
    uint64_t chipid = ESP.getEfuseMac();
    String chipIdString = String((uint32_t)(chipid & 0xFFFF), HEX);
    String deviceName = "Calib-Crutch-BT-" + chipIdString;  // Cria um nome de dispositivo único
    SerialBT.begin(deviceName);    // Inicia Bluetooth com o nome de dispositivo único
  #endif

  #ifdef USE_PIN
    SerialBT.setPin(pin);
    printMessageLn("Usando PIN");
  #endif

  //---- Inicialização do Acelerômetro ----
  if (!cal.begin()) {
    printMessageLn("ERRO 201: Falha ao inicializar o auxiliar de calibração");
  } else if (!cal.loadCalibration()) {
    printMessageLn("Nenhuma calibração carregada/encontrada");
  } else {
    printCalibration();  // Imprime dados de calibração
    delay(3000);
  }

  if (!init_sensors()) {
    printMessageLn("ERRO 202: Falha ao encontrar sensores");
    while (1) delay(10);  // Para se os sensores não forem encontrados
  }

  // Imprime detalhes do sensor
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();    // Configura sensores
  filter.begin(FILTER_UPDATE_RATE_HZ);  // Inicializa o filtro
  timestamp = millis();    // Inicializa timestamp

  Wire.setClock(40000);    // Define o clock I2C para 400KHz

  //---- Inicialização do HX711 ----
  scale.begin(dataPin, clockPin);    // Inicializa o HX711
  EEPROM.begin(512);    // Inicializa EEPROM com tamanho de 512 bytes

  // Carrega calibração da EEPROM
  loadFromMemory();

  printMessageLn("---- HX711 ----");
  scale.set_average_mode();    // Define modo para média
  printMessage("MODO: ");
  printMessageLn(getModeNameHX711(scale.get_mode()));
  printMessage("GANHO: ");
  printMessageLn(getGainNameHX711(scale.get_gain()));
}

//---- Função de Loop Principal ----
//----
void loop() {
  static uint8_t counter = 0;    // Contador para atualizações de impressão
  bool read_flag = false;    // Flag para ler sensores
  bool print_flag = false;    // Flag para imprimir dados
  float gx, gy, gz;    // Variáveis para leituras do giroscópio

  //---- Máquina de Estados Finitos (FSM) ----
  // Verifica cada botão para pressionar
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(keyPadPins[i]) == LOW) {
      // Se o botão for pressionado, muda o estado
      state = static_cast<State>(i);
      // delay(50);  // Atraso de debounce
      break;
    }
  }

  // Trata as ações do estado
  switch (state) {
    case STATE_1:
      // Estado padrão, nada a fazer
      break;
    case STATE_2:
      // Aumenta o número de leituras por medição
      readingsNo++;
      delay(300);  // Previne mudança rápida de estado
      state = STATE_1;  // Retorna ao estado padrão
      break;
    case STATE_3:
      // Reinicia o número de leituras para 1
      readingsNo = 1;
      state = STATE_1;  // Retorna ao estado padrão
      break;
    case STATE_4:
      // Imprime parâmetros da balança e tara a balança
      printScaleParams();
      delay(500);
      scale.tare(20);  // Tara a balança com 20 leituras
      delay(500);
      state = STATE_1;  // Retorna ao estado padrão
      break;
  }

  //---- Leituras do Sensor e Atualização do Filtro ----
  if ((millis() - timestamp) >= (1000 / FILTER_UPDATE_RATE_HZ)) {
    read_flag = true;
    timestamp = millis();
  }

  if (++counter >= PRINT_EVERY_N_UPDATES) {
    print_flag = true;
    digitalWrite(ledPin, !digitalRead(ledPin)); // Alterna estado do LED
    counter = 0;  // Reinicia contador
  }

  if (read_flag) {
    read_flag = false;

    // Lê os sensores de movimento
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);    // Obtém evento do acelerômetro
    gyroscope->getEvent(&gyro);    // Obtém evento do giroscópio
    magnetometer->getEvent(&mag);    // Obtém evento do magnetômetro

    #if defined(AHRS_DEBUG_OUTPUT)
    printMessage("I2C levou "); printMessage(String(millis() - timestamp));  printMessageLn(" ms");
    #endif

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

    #if defined(AHRS_DEBUG_OUTPUT)
    printMessage("Atualização levou ");
    printMessage(String(millis() - timestamp));
    printMessageLn(" ms");

    printMessage("Bruto: ");
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

    // Imprime dados de força e orientação
    printForce_Orientation(filter);

    // printMessage("\n");  // Nova linha

    // Descomente se quiser imprimir quaternion
    // printQuaternion(filter);

    #if defined(AHRS_DEBUG_OUTPUT)
    printMessage("Levou "); printMessage(String(millis() - timestamp, 4)); printMessageLn(" ms");
    #endif
  }

  // Passa dados do Bluetooth para Serial se ambos estiverem definidos
  #if defined(BETTER_PLOTTER) && defined(BT_PRINT)
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
  #endif

  // loopDelay = 1;
  delay(loopDelay);  // Pequeno atraso para evitar resets do watchdog
}

//---- Funções Auxiliares ----

// Função para imprimir mensagens via Bluetooth e/ou Porta Serial
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

// Função para imprimir mensagens com nova linha via Bluetooth e/ou Porta Serial
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

// Função para imprimir dados de calibração
void printCalibration() {
  printMessageLn("Inicialização ou Calibração carregada/encontrada com sucesso");
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

// Função para imprimir parâmetros da balança
void printScaleParams() {
  long offset = scale.get_offset();
  printMessage("Estado: " + String(state));
  printMessage("\tLeituras: " + String(readingsNo));
  printMessage("\tModo da Balança: " + getModeNameHX711(scale.get_mode()));
  printMessage("\tOffset: " + String(offset));
  printMessageLn("");
}

// Função para imprimir dados de força e orientação
void printForce_Orientation(Adafruit_Madgwick filter) {
  float roll, pitch, heading;
  float force;

  // Lê força da balança
  if (scale.is_ready()) {
    force = scale.get_units(readingsNo);  // Obtém a medição atual do HX711
  } else {
    printMessage("Erro 302: Balança não está pronta (atraso: ");
    printMessage(String(loopDelay));
    printMessage("\t ms:");
    printMessage(String(millis()));
    printMessageLn(")");
    // force = 0;  // Define força como zero se a balança não estiver pronta
  }

  // Obtém a orientação do filtro
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  // Imprime dados
  // printMessage("Tempo: " + String(millis()));
  // printMessage("\tF: " + String(force));
  // printMessage("\tOri: ");
  // printMessage("Direção: " + String(heading, 4) + ", ");
  // printMessage("Arfagem: " + String(pitch, 4) + ", ");
  // printMessage("Rolagem: " + String(roll, 4));
  // printMessageLn("");
  
  printMessage(String(millis()));
  printMessage("\tF:");
  printMessage(String(force));
  printMessage(",\tOr: ,");
  printMessage("D:" + String(heading, 2) + ", ");
  printMessage("A:" + String(pitch, 2) + ", ");
  printMessageLn("R:" + String(roll, 2));
}

// Função para imprimir dados de quaternion
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

// Função para obter o nome do modo do HX711
String getModeNameHX711(uint8_t mode) {
  switch (mode) {
    case HX711_AVERAGE_MODE:
      return "Modo Média";
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

// Função para obter o nome do ganho do HX711
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

// Função para ler dados de calibração da EEPROM
void readCalibrationFromEEPROM(float &scaleValue, float &offsetValue) {
  EEPROM.get(calibrationAddress, scaleValue);
  EEPROM.get(calibrationAddress + sizeof(float), offsetValue);
}

// Função para escrever dados de calibração na EEPROM
void writeCalibrationToEEPROM(float scaleValue, float offsetValue) {
  EEPROM.put(calibrationAddress, scaleValue);
  EEPROM.put(calibrationAddress + sizeof(float), offsetValue);
  EEPROM.commit();  // Salva alterações na EEPROM
}

// Função para carregar dados de calibração da memória
void loadFromMemory() {
  float scaleValue, offsetValue;
  readCalibrationFromEEPROM(scaleValue, offsetValue);

  // Verifica se a EEPROM contém dados válidos
  if (isnan(scaleValue) || scaleValue == 0) {
    scaleValue = -5.752855;  // Valor de escala padrão
    printMessageLn("Nenhuma Calibração Encontrada!");
    printMessageLn("Valor de Escala definido: " + String(scaleValue));
  } else {
    printMessageLn("Valor de Escala definido da Memória: " + String(scaleValue));
  }
  scale.set_scale(scaleValue);

  if (isnan(offsetValue)) {
    delay(500);
    scale.tare(20);  // Tara a balança com 20 leituras
    delay(500);
    printMessageLn("Balança tarada!");
  } else {
    printMessageLn("Valor de Offset definido da Memória: " + String(offsetValue));
    scale.set_offset(offsetValue);
  }
}