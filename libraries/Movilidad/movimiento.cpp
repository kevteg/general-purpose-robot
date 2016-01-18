#include "movimiento.h"
robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo) :
 motor_derecho(pin_derecho), motor_izquierdo(pin_izquierdo){
	this->pin_derecho = pin_derecho;
	this->pin_izquierdo = pin_izquierdo;
  this->pin_en_derecho = pin_en_derecho;
	this->pin_en_izquierdo = pin_en_izquierdo;
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
	setVelocidad(minVelo);
  detener();
}

robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo, int velocidad) :
 motor_derecho(pin_derecho, velocidad), motor_izquierdo(pin_izquierdo, velocidad){
	this->pin_derecho = pin_derecho;
	this->pin_izquierdo = pin_izquierdo;
  this->pin_en_derecho = pin_en_derecho;
	this->pin_en_izquierdo = pin_en_izquierdo;
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
	setVelocidad(velocidad);
  detener();
}
void robot::movilidad::activarSensores(){
  pinMode(pin_en_derecho, OUTPUT);    // make line an output
  pinMode(pin_en_izquierdo, OUTPUT);    // make line an output
  digitalWrite(pin_en_derecho, HIGH); // drive line high
  digitalWrite(pin_en_izquierdo, HIGH); // drive line high

  delayMicroseconds(10);      // charge line

  pinMode(pin_en_derecho, INPUT);     // make line an input
  pinMode(pin_en_izquierdo, INPUT);     // make line an input
  tiempo_ini_en_der = micros();
  tiempo_ini_en_izq = micros();
}
void robot::movilidad::verificarVelocidades(){
    if(ult_mov != mov_detener){
      if(tiempo_en_der){
        if(tiempo_en_der > min_v_blanco_en_der && tiempo_en_der < max_v_blanco_en_der && !v){
            n++;
           v = true;
        }else if(tiempo_en_der > min_v_negro_en_der && tiempo_en_der < max_v_negro_en_der && v)
           v = false;
        //Serial.println(tiempo_en_der);
        tiempo_en_der = 0;
      }

      if(tiempo_en_izq){
        if(tiempo_en_izq > min_v_blanco_en_izq && tiempo_en_izq < max_v_blanco_en_izq && !v1){
           n1++;
           v1 = true;
        }else if(tiempo_en_izq > min_v_negro_en_izq && tiempo_en_izq < max_v_negro_en_izq && v)
           v1 = false;
        //Serial.println(tiempo_en_izq);
        tiempo_en_izq = 0;
      }

      double di_d = (c_movimiento*n) / 6;
      double di_i = (c_movimiento*n1) / 6;
      tiempo = millis() - tiempo_inicio;
      double v_d = di_d / (tiempo / 1000);
      double v_i = di_i / (tiempo / 1000);
      tiempo_v = millis() - tiempo_inicio_v;
      if(tiempo_v > 1000)
        rectificacion(v_d, v_i);
      Serial.print(v_d);
      Serial.print(" ");
      Serial.print(v_i);
      Serial.println(" cm / s derecha e izquierda");
    }
}
void robot::movilidad::rectificacion(double v_r_der, double v_r_izq){
  int p = 5;
  if(abs(v_r_der - v_r_izq) > 1){
  Serial.print("Entro");
    if(v_r_der > v_r_izq){
      Serial.println(" 1");
      //int p = (static_cast<int>((100*v_r_izq)/v_r_der)) / 50;

      vel_izq += (vel_izq + p <= 255)?p:0;
      vel_der -= (vel_der - p >= 55)?p:0;
      Serial.print(vel_der);
      Serial.print(" ");
      Serial.println(vel_izq);
    }else{
      Serial.println(" 2");
      //int p = (static_cast<int>((100*v_r_der)/v_r_izq)) / 50;

      vel_der += (vel_der + p <= 255)?p:0;
      vel_izq -= (vel_izq - p >= 55)?p:0;
      Serial.print(vel_der);
      Serial.print(" ");
      Serial.println(vel_izq);
    }
    actMov();
  }
  tiempo_inicio_v = millis();
}
void robot::movilidad::conteo_en_der() {
  tiempo_en_der = micros() - tiempo_ini_en_der;
}

void robot::movilidad::conteo_en_izq() {
  tiempo_en_izq = micros() - tiempo_ini_en_izq;
}

void robot::movilidad::derecha(){
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_derecha;
  vel_der = velocidad;
  vel_izq = velocidad;
  actMov();
}

void robot::movilidad::izquierda(){
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_izquierda;
  vel_der = velocidad;
  vel_izq = velocidad;
  actMov();
}

void robot::movilidad::adelante(){
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_adelante;
  //vel_der = velocidad;
  //vel_izq = velocidad;
  actMov();

}

void robot::movilidad::atras(){
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_atras;
  vel_der = velocidad;
  vel_izq = velocidad;
  actMov();
}

void robot::movilidad::detener(){
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
  ult_mov = mov_detener;
  //vel_der = velocidad;
  //vel_izq = velocidad;
  actMov();

}

void robot::movilidad::actMov(){
  int vel_d = (ult_mov != mov_detener)?((ult_mov == mov_adelante || ult_mov == mov_izquierda)?vel_der:-vel_der):noVelocidad;
  int vel_i = (ult_mov != mov_detener)?((ult_mov == mov_adelante || ult_mov == mov_derecha)?vel_izq:-vel_izq):noVelocidad;

  motor_derecho.setSpeed(vel_d);
  motor_izquierdo.setSpeed(vel_i);
}

int robot::movilidad::getPinEnDer(){
  return pin_en_derecho;
}
int robot::movilidad::getPinEnIzq(){
  return pin_en_izquierdo;
}

inline void robot::movilidad::setVelocidad(int nueva_velocidad){
	velocidad = (nueva_velocidad >= minVelo && nueva_velocidad <= maxVelo)?nueva_velocidad:velocidad;
  vel_der = velocidad;
  vel_izq = velocidad;
  n = 0;
  n1 = 0;
  v = false;
  v1 = false;
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
