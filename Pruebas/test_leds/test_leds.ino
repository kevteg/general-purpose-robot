#include <leds.h>
#define number_leds 3
int array_leds[3];
int combinacion[3];

int n;
using namespace robot;
control_leds *leds;
void setup(){
  array_leds[0] = 9;
  array_leds[1] = 10;
  array_leds[2] = 18;
  combinacion[0] = 9;
  combinacion[1] = 10;
  combinacion[2] = 18;
  leds = new control_leds(array_leds, number_leds);
  
  Serial.begin(9600);
   leds->combinacion(combinacion, 3, 500);
  n = 0;
}

void loop(){
 /* if(Serial.available() > 0){
    n = Serial.read();
    leds->mostrarComportamiento(n);
  }*/
  leds->combinacion(combinacion, 3, 500);
}
