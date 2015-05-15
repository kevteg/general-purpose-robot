#include "movimiento.h"
robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo) : 
 motor_derecho(pin_derecho), motor_izquierdo(pin_izquierdo){
	this->pin_derecho = pin_derecho;
	this->pin_izquierdo = pin_izquierdo;
	setVelocidad(maxVelo);
}

robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int velocidad) :
 motor_derecho(pin_derecho, velocidad), motor_izquierdo(pin_izquierdo, velocidad){
	this->pin_derecho = pin_derecho;
	this->pin_izquierdo = pin_izquierdo;
	setVelocidad(velocidad);
}

void robot::movilidad::derecha(){
	motor_derecho.setSpeed(-velocidad);
	motor_izquierdo.setSpeed(velocidad);
}

void robot::movilidad::izquierda(){
	motor_derecho.setSpeed(velocidad);
	motor_izquierdo.setSpeed(-velocidad);
}

void robot::movilidad::adelante(){
	motor_derecho.setSpeed(velocidad);
	motor_izquierdo.setSpeed(velocidad);
}

void robot::movilidad::atras(){
	motor_derecho.setSpeed(-velocidad);
	motor_izquierdo.setSpeed(-velocidad);
}

void robot::movilidad::detener(){
	motor_derecho.setSpeed(noVelocidad);
	motor_izquierdo.setSpeed(noVelocidad);
}

inline void robot::movilidad::setVelocidad(int nueva_velocidad){
	velocidad = (nueva_velocidad>=minVelo && nueva_velocidad<= maxVelo)?nueva_velocidad:velocidad;
}
