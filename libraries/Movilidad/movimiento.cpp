#include "movimiento.h"
robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo) :
 motor_derecho(pin_derecho), motor_izquierdo(pin_izquierdo){
	this->pin_derecho = pin_derecho;
	this->pin_izquierdo = pin_izquierdo;
	setVelocidad(maxVelo);
  detener();
}

robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int velocidad) :
 motor_derecho(pin_derecho, velocidad), motor_izquierdo(pin_izquierdo, velocidad){
	this->pin_derecho = pin_derecho;
	this->pin_izquierdo = pin_izquierdo;
	setVelocidad(velocidad);
  detener();
}

void robot::movilidad::derecha(){
	motor_derecho.setSpeed(-velocidad);
	motor_izquierdo.setSpeed(velocidad);
  ult_mov = mov_derecha;
}

void robot::movilidad::izquierda(){
	motor_derecho.setSpeed(velocidad);
	motor_izquierdo.setSpeed(-velocidad);
  ult_mov = mov_izquierda;
}

void robot::movilidad::adelante(){
	motor_derecho.setSpeed(velocidad);
	motor_izquierdo.setSpeed(velocidad);
  ult_mov = mov_adelante;
}

void robot::movilidad::atras(){
	motor_derecho.setSpeed(-velocidad);
	motor_izquierdo.setSpeed(-velocidad);
  ult_mov = mov_atras;
}

void robot::movilidad::detener(){
	motor_derecho.setSpeed(noVelocidad);
	motor_izquierdo.setSpeed(noVelocidad);
  ult_mov = mov_detener;
}

inline void robot::movilidad::setVelocidad(int nueva_velocidad){
	velocidad = (nueva_velocidad >= minVelo && nueva_velocidad <= maxVelo)?nueva_velocidad:velocidad;
  switch(ult_mov){
    case mov_adelante:
      adelante();
    break;
    case mov_atras:
      atras();
    break;
    case mov_derecha:
      derecha();
    break;
    case mov_izquierda:
      izquierda();
    break;
    case mov_detener:
      detener();
    break;
  }
}
