/**
 *
 * HX711 library for Arduino - example file
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include "HX711.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 17;
const int LOADCELL_SCK_PIN = 18;



HX711 scale;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(105.5f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();				        // reset the scale to 0

}

void loop() {
  Serial.print("one reading:\t");
  Serial.print(scale.get_units(), 3);
  Serial.print("\t| average:\t");
  Serial.println(scale.get_units(10), 3);

  //scale.power_down();			        // put the ADC in sleep mode
  delay(1000);
  //scale.power_up();
  //delay(100);
}
