#include <AFMotor.h>
#include <adafruit_control_motor.h>
#include <control_motor.h>
#include <movimiento.h>

#define puertoDerecho 1
#define puertoIzquierdo 2
#define velocidad 55

using namespace robot;


movilidad mover(puertoDerecho, puertoIzquierdo, velocidad);

void setup(){
  Serial.begin(9600);
  decide('5');
}

void loop(){
  if(Serial.available() > 0){
    char lec = Serial.read();
    decide(lec);
  }
}

void decide(char lec){

  switch(lec){
    case '1':
      mover.adelante();
      Serial.println("Robot hacia adelante");
    break;
    case '2':
      mover.atras();
      Serial.println("Robot hacia atras");
    break;
    case '3':
      mover.derecha();
      Serial.println("Robot girando a la derecha");
    break;
    case '4':
      mover.izquierda();
      Serial.println("Robot girando a la izquierda");
    break;
    case '5':
      mover.detener();
      Serial.println("Robot detenido");
    break;
    default:
      Serial.println("No existe opcion"+lec);
    break;
  }

}


