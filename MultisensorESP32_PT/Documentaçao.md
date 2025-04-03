# Criar arquivo markdown a partir do conteúdo do documento
markdown_content = """# Documentação do Sistema MultisensorESP32_PT

## Bibliotecas Utilizadas

A tabela abaixo lista todas as bibliotecas utilizadas no projeto MultisensorESP32_PT, suas funções e onde são utilizadas no código:

| Biblioteca | Link | Função | Linhas de uso no código |
|------------|------|--------|------------------------|
| HX711 | [GitHub](https://github.com/bogde/HX711) | Gerencia o amplificador de célula de carga para medir força/peso |   |
| BluetoothSerial | [GitHub](https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial) | Permite a comunicação Bluetooth no ESP32 |   |
| esp_system.h | [GitHub](https://github.com/espressif/arduino-esp32) | Fornece funções do sistema para o ESP32 | Linha 3, 119 (ESP.getEfuseMac()) |
| Adafruit_Sensor_Calibration | [GitHub](https://github.com/adafruit/Adafruit_Sensor_Calibration) | Gerencia a calibração de sensores |   |
| Adafruit_AHRS | [GitHub](https://github.com/adafruit/Adafruit_AHRS) | Implementa filtros de orientação (Madgwick/Mahony) |   |
| EEPROM | [GitHub](https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/EEPROM) | Permite salvar dados na memória não volátil |   |
| Wire | [Arduino](https://www.arduino.cc/en/reference/wire) | Facilita a comunicação I2C |   |
| NXP_FXOS_FXAS.h | (arquivo personalizado) | Inicializa e configura os sensores inerciais NXP | Linha 36 (inclusão), Linhas 142-146 (chamada a init_sensors()), Linha 152 (chamada a setup_sensors()) |
| Adafruit_FXAS21002C | [GitHub](https://github.com/adafruit/Adafruit_FXAS21002C) | Controla o giroscópio FXAS21002C | Incluída dentro de NXP_FXOS_FXAS.h |
| Adafruit_FXOS8700 | [GitHub](https://github.com/adafruit/Adafruit_FXOS8700) | Controla o acelerômetro e magnetômetro FXOS8700 | Incluída dentro de NXP_FXOS_FXAS.h |

## Repositório do Projeto

O código completo do projeto está disponível no GitHub:
[Smart_Crutches/MultisensorESP32_PT](https://github.com/Smart_Crutches/MultisensorESP32_PT)

## Biblioteca Personalizada NXP_FXOS_FXAS.h

Na pasta do projeto, você encontrará o arquivo NXP_FXOS_FXAS.h, que é uma biblioteca personalizada para facilitar a inicialização e configuração dos sensores inerciais NXP (FXOS8700 e FXAS21002C). Esta biblioteca:

1. Encapsula a inicialização dos sensores FXOS8700 (acelerômetro e magnetômetro) e FXAS21002C (giroscópio)
2. Configura os endereços I2C corretos para os sensores
3. Fornece ponteiros para os sensores que são utilizados pelo programa principal
4. Simplifica o acesso aos dados dos sensores através de uma interface unificada

A biblioteca deve ser incluída após a declaração dos ponteiros para os sensores Adafruit, conforme indicado nos comentários do código.

## Funcionalidades Principais

O sistema MultisensorESP32_PT integra:

- Medição de força através de célula de carga (HX711)
- Detecção de orientação usando sensores inerciais (acelerômetro, giroscópio e magnetômetro)
- Comunicação Bluetooth para transmissão de dados
- Máquina de estados para controle de operação
- Calibração e armazenamento de dados na EEPROM


