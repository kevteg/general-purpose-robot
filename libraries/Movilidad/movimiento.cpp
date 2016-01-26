#include "movimiento.h"

robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo, int velocidad) :
 motor_derecho(pin_derecho),
 motor_izquierdo(pin_izquierdo),
 pin_en_derecho(pin_en_derecho),
pin_en_izquierdo(pin_en_izquierdo){
  Kp_i = 0.8;
  Kd_i = 1.4;
  Ki_i = 0;
  Kp_d = 0.55;
  Kd_d = 1.1;
  Ki_d = 0;
  pwm_der = 0;
  pwm_izq = 0;
  last_error_d = 0;
  last_error_i = 0;
  integral_d = 0;
  integral_i = 0;
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

    tiempo_v = millis() - tiempo_inicio_v;
    if(tiempo_v > tiempo_verificacion){
      double velocidad_d = calculoVelocidad(1);
      double velocidad_i = calculoVelocidad(2);
      distancia_promedio = (distancia_d + distancia_i)/2;
      /*A partir de 30 cm es muy posible que falle gracias a los sensores, por lo que se reinicia*/
      /*pwm_der = updatePid(Kp_d, Kd_d, Ki_d, pwm_der, velocidad_requerida, (baux && (millis() - tiempo_inicio) > 1000)?va_d:velocidad_d, &last_error_d, &integral_d);
      pwm_izq = updatePid(Kp_i, Kd_i, Ki_i,pwm_izq, velocidad_requerida, (baux && (millis() - tiempo_inicio) > 1000)?va_i:velocidad_i, &last_error_i, &integral_i);*/
      pwm_der = updatePid(Kp_d, Kd_d, Ki_d, pwm_der, velocidad_requerida, velocidad_d, &last_error_d, &integral_d);
      pwm_izq = updatePid(Kp_i, Kd_i, Ki_i,pwm_izq, velocidad_requerida, velocidad_i, &last_error_i, &integral_i);
      if(distancia_promedio > 30){
        conteo_izq = 0;
        conteo_der = 0;
        tiempo_inicio = millis();
        va_d = velocidad_d;
        va_i = velocidad_i;
        last_error_d = 0;
        last_error_i = 0;
        integral_d = 0;
        integral_i = 0;
        Serial.println("##########################################################################");
      }



      Serial.print("----------------------------------------------------------------");
      Serial.print(pwm_der);
      Serial.print(" ");
      Serial.println(pwm_izq);

      actMov();
      tiempo_inicio_v = millis();
    }
  }
}
double robot::movilidad::calculoVelocidad(int mot){
  double velocidad;
  tiempo = millis() - tiempo_inicio;

  if(mot == 1){
    distancia_d = ((c_movimiento / altos_rueda)*conteo_der);
    velocidad =  (distancia_d / ((tiempo) / double(1000)));
  }else{
    distancia_i = ((c_movimiento / altos_rueda)*conteo_izq);
    velocidad =   (distancia_i / ((tiempo) / double(1000)));
  }

  return velocidad;
}
int robot::movilidad::updatePid(double Kp, double Kd, double Ki, int pwm_act, int vel_req, double vel_act, double* last_error, double *integral){
  double error = 0;
  double term_pid = 0;
  error = static_cast<double>(vel_req) - vel_act;
  //if(abs(error) > 1){
    double tp, td, ti;
    //Control Pid
    *integral += Ki*(static_cast<double>(tiempo_verificacion) / static_cast<double>(1000))*(error);
    tp = (Kp * error);
    ti = *integral;
    td = Kd/(static_cast<double>(tiempo_verificacion) / static_cast<double>(1000)) * (error - *last_error);

    term_pid = tp
              + ti
              + td;

    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("vel req: ");
    Serial.println(vel_req);
    Serial.print("vel act: ");
    Serial.println(vel_act);

    Serial.print("Tp: ");
    Serial.println(tp);
    Serial.print("Ti: ");
    Serial.println(ti);
    Serial.print("Td: ");
    Serial.println(td);
    Serial.print("tpid: ");
    Serial.println(term_pid);
    Serial.println("---------");
    *last_error = error;
  //}
  return constrain(pwm_act + int(term_pid), 10, 255);
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
  /*conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;*/

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
/*double robot::movilidad::getKp(){
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
}*/

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
