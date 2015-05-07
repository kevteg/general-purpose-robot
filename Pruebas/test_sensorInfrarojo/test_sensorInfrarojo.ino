#include <QTRSensors.h>

#include <qtr_control_sensor.h>
#define pinS1 2
#define pinS2 3
#define numeroSensores 2
using namespace robot;
qtr_sensor sensor_infra(pinS1, pinS2);
void setup(){
  sensor_infra.calibrar();
  Serial.begin(9600);
}
void loop(){
// unsigned int posicion = sensor_infra.lectura(); //Al realizar la lectura se guardan los datos en el vector de informacion
 
 for(int i = 0; i < numeroSensores; i++){
   Serial.print(sensor_infra.valorSensor(i));
   Serial.print('\t');
 }
 Serial.print('\n');
/* Serial.print('Posicion: ');
 Serial.print(posicion);
  */ 
}

