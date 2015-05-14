
/*Movilidad*/
#include <AFMotor.h>
#include <adafruit_control_motor.h>
#include <control_motor.h>
#include <movimiento.h>
/*Sensores*/
#include <NewPing.h>
#include <QTRSensors.h>
#include <control_sensor.h>
#include <newping_control_sensor.h>
#include <qtr_control_sensor.h>
/*Comunicación*/
/*Auxiliares*/

#include <promedio.h>

#define puertom_d 3
#define puertom_i 4
#define pin_trigger 14
#define pin_echo 15
#define sensori_i 2
#define sensori_d 3
#define velocidad 100
#define maxdistancia_ult 100
#define nombre_r 'A' //A es Ocho, B es Póker


#include <conducta.h>
using namespace robot;

comportamiento Robot(nombre_r, puertom_d, puertom_i, velocidad, pin_trigger, pin_echo, maxdistancia_ult, sensori_d, sensori_i);

void setup(){
  //Serial.begin(9600);
 Robot.inicializar();
}
void loop(){
  //String a = (String)delimitador_i;
  //String b = (String)delimitador_f;
  
 
   Robot.run();
}



