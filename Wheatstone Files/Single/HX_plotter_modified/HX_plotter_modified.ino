//
//    FILE: HX_plotter.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: HX711 demo
//     URL: https://github.com/RobTillaart/HX711


#include "HX711.h"

HX711 scale;

const uint8_t dataPin = 34;
const uint8_t clockPin = 23;

uint8_t No_readings = 1;
uint16_t dly = 5;
uint8_t cont = 0;

uint32_t start, stop;
volatile float f;

unsigned long startTime = 0 ;
unsigned long endTime = 0;
unsigned long duration = 0;

void setup()
{
  Serial.begin(250000);
  Serial.println("Programa:\n\t HX_plotter_modified.ino");
  scale.begin(dataPin, clockPin);

  scale.set_scale(-5.752855);  // Set the scale

  delay(500);
  scale.tare(20);
  delay(500);
}

void loop(){

  for (int i = 1; i < 15; i++){//number of samples
    for (int j = 1; j< 15; j++){ //delay in ms

        if (scale.is_ready()) {
        
        endTime = micros();
        duration = endTime - startTime; 
        String output = String("\n" + String(f) + "\t" +
                              String((float)duration/1000.0f) + "\t" +
                              String(i) +  "\t" + 
                              String(j) +  "\t" + " ");
        Serial.print(output);
        startTime = micros();
        f = scale.get_units(i);
        // No_readings = (No_readings++)%15 + 1;
        // dly = (dly++)%15 + 1;
        // cont=0;
      }else{
        // String str = String(String(dly*(++cont)) + ", ");
        // Serial.print(str);
        Serial.print("-");
        // delay(500);
      }
      delay(j);
    }
  } 
}


// -- END OF FILE --

