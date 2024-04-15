//
//    FILE: HX_plotter.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: HX711 demo
//     URL: https://github.com/RobTillaart/HX711


#include "HX711.h"

HX711 scale;
uint8_t dataPin  = 34;
uint8_t clockPin = 23;

uint32_t start, stop;
volatile float f;


void setup()
{
  Serial.begin(115200);
  scale.begin(dataPin, clockPin);
  //scale.set_offset(4293645371); 
  scale.set_scale(-4.868363);
  // scale.set_average_mode();
  // Serial.println(scale.get_mode());
  //scale.tare();
}


void loop()
{
  // continuous scale 4x per second
  f = scale.get_units(5);
  Serial.println(f);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(110);
}


// -- END OF FILE --

