//
//    FILE: HX_plotter_multimap.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: HX711 demo
//     URL: https://github.com/RobTillaart/HX711



#include "MultiMap.h"

// inkg[] holds the weights from the linear calibrated HX711 sensor
float inkg[]  = {0,    a,    b,  a+b,    c,  a+c,  b+c, a+b+c};
        

float a=0.0f, b=1246.0f, c=5060.0f;
// outkg[] holds the corrected weight
float outkg[] = {0,    a,    b,  a+b,    c,  a+c,  b+c, a+b+c};
            //  {0, 1246, 2490, 3736, 5060, 6306, 7550, 8796};


#include "HX711.h"

HX711 scale;

uint8_t dataPin  = 34;
uint8_t clockPin = 23;

uint32_t start, stop;

float raw, corrected;


void setup()
{
  Serial.begin(115200);
  // Serial.println(__FILE__);
  // Serial.print("LIBRARY VERSION: ");
  // Serial.println(HX711_LIB_VERSION);
  Serial.println();

  scale.begin(dataPin, clockPin);

  // TODO find a nice solution for this calibration..
  // load cell factor 20 KG
  // scale.set_scale(127.15);
  // load cell factor 5 KG
  scale.set_scale(420.0983);       // TODO you need to calibrate this yourself.
  // reset the scale to zero = 0
  scale.tare();

  //  dump whole range 0..50 kg
  for (float w = 0; w <= 50; w += 0.1)
  {
    Serial.print(w, 3);
    Serial.print('\t');
    corrected = multiMap<float>(w, inkg, outkg, 9);  //  9 is the size of the arrays!
    Serial.println(corrected, 3);
  }

  delay(50000);

}


void loop()
{
  raw = scale.get_units(5);
  corrected = multiMap<float>(raw, inkg, outkg, 9);  //  9 is the size of the arrays!

  Serial.print(raw, 3);
  Serial.print('\t');
  Serial.println(corrected, 3);
  delay(250);
}


// -- END OF FILE --
