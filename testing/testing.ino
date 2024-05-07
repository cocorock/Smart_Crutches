#include "HX711.h"

// Define the data and clock pins for the HX711
uint8_t dataPin = 34;
uint8_t clockPin = 23;
uint8_t ledPin = 5;

HX711 scale;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  scale.begin(dataPin, clockPin);
  scale.set_scale(-5.752855);  // Set the scale
  scale.set_offset(-1314026);  // Set the offset
  scale.begin(dataPin, clockPin);
  delay(500);
  scale.tare(10);
  delay(500);
}

volatile float reading = 0;
long start_time = millis();
long duration = 23;
long timeout = 15;
long del = 100;
uint8_t acc = 0;
double op = 1.00000001;

void loop() {
  for(int i=1; i<16; i++){
    delay(200);
    unsigned long startTime = millis();  // Start time
    float measurement = scale.get_units(i);
    // float measurement = scale.read();  //scale.get_units(i);
    while (!scale.is_ready()) {  // Wait until the scale is ready
      // Optional delay to prevent the loop from running too fast
      // delay(5);
      digitalWrite(ledPin, !digitalRead(ledPin));
    }
    unsigned long endTime = millis();  // End time
    unsigned long elapsedTime = endTime - startTime;

    // Serial.println("");
    Serial.print(i);
    Serial.print("  Time: ");
    Serial.print(elapsedTime);
    Serial.print(" ms   ");
    Serial.print("  T p Unit: ");
    Serial.print(elapsedTime/(1.0*i));
    Serial.print(" ms   ");
    Serial.print(" F ");
    Serial.print((1.0*i)/elapsedTime);
    Serial.print(" hz   ");
    Serial.print("Measurement: ");
    Serial.println(measurement);
  }
   delay(10000);
}


// void loop() {
//   for (int i = 1; i < 16; i++) {
//     delay(del);
//     acc=0;
//     start_time = millis();
//     while ((millis() - start_time) < timeout) {
//       if (scale.is_ready()) {
//         reading = scale.get_units(i);  // Read the data from the scale
//         duration = millis() - start_time;
//         break;
//       }else{
//         Serial.print(" . ");
//         Serial.print(millis());
//         // delay(1);
//       }
//     }

//     Serial.println("");
//     Serial.print(reading);
//     Serial.print(",\t");
//     Serial.print(i);
//     Serial.print(",\t");
//     Serial.print(duration);
//     Serial.print(" ms");
//   }
//   Serial.print("\n");
//   delay(1000);  // Delay for a second

//   for (int i = 1; i < 6; i++) {
//     delay(del);
//     acc=0;
//     start_time = millis();
//     while ((millis() - start_time) < timeout) {
//       if (scale.is_ready()) {
//         reading = scale.read();  // Read the data from the scale
//         duration = millis() - start_time;
//         break;
//       }else{
//         Serial.print(" . ");
//         Serial.print(millis());
//         // delay(1);
//       }
//     }

//     Serial.println("");
//     Serial.print(reading);
//     Serial.print(",\t");
//     Serial.print(i);
//     Serial.print(",\t");
//     Serial.print(duration);
//     Serial.print(" ms");
//   }
//   Serial.print("\n");

//   Serial.print("\n\n");
//   delay(15000);
// }
