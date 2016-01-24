#include "movimiento.h"

robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo, int velocidad) :
 motor_derecho(pin_derecho),
 motor_izquierdo(pin_izquierdo),
 pin_en_derecho(pin_en_derecho),
pin_en_izquierdo(pin_en_izquierdo){
  Kp = 0.5;
  Kd = 0.5;
  Ki = 0.5;
  detener();
  setVelocidad(velocidad);
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
void robot::movilidad::conteoRevoluciones(){
    if(ult_mov != mov_detener){
      if(tiempo_en_der){
        if(tiempo_en_der > min_v_blanco_en_der && tiempo_en_der < max_v_blanco_en_der && !v){
            conteo_der++;
           v = 1;
        }else if(tiempo_en_der > min_v_negro_en_der && tiempo_en_der < max_v_negro_en_der && v)
           v = 0;
        tiempo_en_der = 0;
      }

      if(tiempo_en_izq){
        if(tiempo_en_izq > min_v_blanco_en_izq && tiempo_en_izq < max_v_blanco_en_izq && !v1){
           conteo_izq++;
           v1 = 1;
        }else if(tiempo_en_izq > min_v_negro_en_izq && tiempo_en_izq < max_v_negro_en_izq && v1)
           v1 = 0;
        tiempo_en_izq = 0;
      }
    }
}
void robot::movilidad::proceso(){

  if(ult_mov != mov_detener){
    double* velocidades = calculoVelocidad();
    tiempo_v = millis() - tiempo_inicio_v;
    if(tiempo_v > tiempo_verificacion){


      pwm_der = updatePid(pwm_der, velocidad_requerida, velocidades[0], &last_error_d, &integral_d);
      pwm_izq = updatePid(pwm_izq, velocidad_requerida, velocidades[1], &last_error_i, &integral_i);

      Serial.print("----------------------------------------------------------------");
      Serial.print(pwm_der);
      Serial.print(" ");
      Serial.println(pwm_izq);

      actMov();
      tiempo_inicio_v = millis();
    }
  }
}
double* robot::movilidad::calculoVelocidad(){
  double velocidades[2];
  tiempo = millis() - tiempo_inicio;
  velocidades[0] =  ((c_movimiento / altos_rueda)*conteo_der) / ((tiempo) / double(1000));
  velocidades[1] =   ((c_movimiento / altos_rueda)*conteo_izq) / ((tiempo) / double(1000));

  Serial.print(  velocidades[0]);
  Serial.print(" ");
  Serial.println(  velocidades[1]);


//  conteo_der = 0;
//  conteo_izq = 0;
  return velocidades;
}
int robot::movilidad::updatePid(int pwm_act, int vel_req, double vel_act, double* last_error, double *integral){
  float term_pid = 0;                                                           
  double error=0;
  error = static_cast<double>(vel_req) - vel_act;
  //Control Pid
  *integral += Ki*(error*(tiempo_verificacion / 1000));
  term_pid = (Kp * error)
            + *integral
            + ((Kd/(tiempo_verificacion / 1000)) * (error - *last_error));

  *last_error = error;
  return constrain(pwm_act + int(term_pid), 55, 255);
}
void robot::movilidad::conteo_en_der() {
  tiempo_en_der = micros() - tiempo_ini_en_der;
}

void robot::movilidad::conteo_en_izq() {
  tiempo_en_izq = micros() - tiempo_ini_en_izq;
}

void robot::movilidad::derecha(){
  conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;
  pwm_der = 0;
  pwm_izq = 0;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_derecha;
  actMov();
}

void robot::movilidad::izquierda(){
  conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;
  pwm_der = 0;
  pwm_izq = 0;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_izquierda;
  actMov();
}

void robot::movilidad::adelante(){
  conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;
  last_error_d = 0;
  last_error_i = 0;
  integral_d = 0;
  integral_i = 0;
  pwm_der = 0;
  pwm_izq = 0;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_adelante;
  actMov();
}

void robot::movilidad::atras(){
  conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;
  pwm_der = 0;
  pwm_izq = 0;
  tiempo_inicio_v = millis();
  tiempo_inicio = millis();
  ult_mov = mov_atras;

  actMov();
}

void robot::movilidad::detener(){
  conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;
  ult_mov = mov_detener;
  actMov();
}

void robot::movilidad::actMov(){
  int pwm_d = (ult_mov != mov_detener)?((ult_mov == mov_adelante || ult_mov == mov_izquierda)?pwm_der:-pwm_der):zero_pwm;
  int pwm_i = (ult_mov != mov_detener)?((ult_mov == mov_adelante || ult_mov == mov_derecha)?pwm_izq:-pwm_izq):zero_pwm;
  motor_izquierdo.setSpeed(pwm_i);
  motor_derecho.setSpeed(pwm_d);
}

int robot::movilidad::getPinEnDer(){
  return pin_en_derecho;
}
int robot::movilidad::getPinEnIzq(){
  return pin_en_izquierdo;
}
double robot::movilidad::getKp(){
  return Kp;
}
double robot::movilidad::getKi(){
  return Ki;
}
double robot::movilidad::getKd(){
  return Kd;
}
void robot::movilidad::setKp(double Kp){
  if(Kp > 0)
    this->Kp = Kp;
}
void robot::movilidad::setKi(double Ki){
  if(Ki > 0)
    this->Ki = Ki;
}
void robot::movilidad::setKd(double Kd){
  if(Kd > 0)
    this->Kd = Kd;
}
void robot::movilidad::setConstantesPid(double Kp, double Ki, double Kd){
  setKp(Kp);
  setKi(Ki);
  setKd(Kd);
}

inline void robot::movilidad::setVelocidad(int nueva_velocidad){
	velocidad_requerida = (nueva_velocidad >= min_velo && nueva_velocidad <= max_velo)?nueva_velocidad:velocidad_requerida;

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
