#include "movimiento.h"

robot::movilidad::movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo, int velocidad) :
 motor_derecho(pin_derecho),
 motor_izquierdo(pin_izquierdo),
 pin_en_derecho(pin_en_derecho),
pin_en_izquierdo(pin_en_izquierdo){
  Kp_i = 0.8;
  Kd_i = 1.7;
//Ki_i = 0;
  Kp_d = 0.4;
  Kd_d = 0.75;
//Ki_d = 0;
  pwm_der = 0;
  pwm_izq = 0;
  detener();
  setVelocidad(velocidad);
}
void robot::movilidad::activarSensores(){
  if(ult_mov != mov_detener){
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
      distancia_promedio_total = distancia_anterior + distancia_promedio;
      switch (ult_mov) {
        case mov_adelante:
        case mov_atras:
          if(distancia_promedio_total > distancia_recorrer)
            detener();
          else
            control(velocidad_d, velocidad_i, distancia_promedio, velocidad_requerida);
        break;
        case mov_derecha:
        case mov_izquierda:
          tiempo_giro = millis() - tiempo_inicio_giro;
          #if debug
            Serial.println("Ver vuelta");
            Serial.print("tiempo de giro: ");
            Serial.println(tiempo_giro);
          #endif
          if(tiempo_giro + tiempo_verificacion >= tiempo_por_girar*1000)
            detener();
          else
            control(velocidad_d, velocidad_i, distancia_promedio, velocidad_giro);
        break;
      }
      tiempo_inicio_v = millis();
    }
  }
}

void robot::movilidad::control(double velocidad_d, double velocidad_i, double distancia_promedio, double velocidad_requerida){
  /*A partir de 30 cm es muy posible que falle gracias a los sensores, por lo que se reinicia*/
 pwm_der = updatePid(Kp_d, Kd_d, pwm_der, velocidad_requerida, velocidad_d, &last_error_d);
 pwm_izq = updatePid(Kp_i, Kd_i, pwm_izq, velocidad_requerida, velocidad_i, &last_error_i);
 #if reinicio_30_cm
   if(distancia_promedio > distancia_reinicio){
      conteo_izq = 0;
      conteo_der = 0;
      tiempo_inicio = millis();
      last_error_d = 0;
      last_error_i = 0;
      distancia_anterior = distancia_promedio;
      #if debug
        Serial.println("##########################################################################");
      #endif
    }
  #endif

  #if debug
    Serial.print("----------------------------------------------------------------");
    Serial.print(pwm_der);
    Serial.print(" ");
    Serial.println(pwm_izq);
  #endif
  actMov();
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
int robot::movilidad::updatePid(double Kp, double Kd, int pwm_act, int vel_req, double vel_act, double* last_error){
  double error = 0;
  double term_pid = 0;
  error = static_cast<double>(vel_req) - vel_act;
  double tp, td;
  //Control Pid
  //*integral += Ki*(static_cast<double>(tiempo_verificacion) / static_cast<double>(1000))*(error);
  tp = (Kp * error);
  //ti = *integral;
  td = Kd/(static_cast<double>(tiempo_verificacion) / static_cast<double>(1000)) * (error - *last_error);

  term_pid = tp
            + td;
  #if debug
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("vel req: ");
    Serial.println(vel_req);
    Serial.print("vel act: ");
    Serial.println(vel_act);

    Serial.print("Tp: ");
    Serial.println(tp);
    Serial.print("Td: ");
    Serial.println(td);
    Serial.print("tpid: ");
    Serial.println(term_pid);
    Serial.println("---------");
  #endif
  *last_error = error;
  return constrain(pwm_act + int(term_pid), 10, 255);
}
void robot::movilidad::conteo_en_der() {
  tiempo_en_der = micros() - tiempo_ini_en_der;
}

void robot::movilidad::conteo_en_izq() {
  tiempo_en_izq = micros() - tiempo_ini_en_izq;
}

void robot::movilidad::derecha(double theta){
  tiempo_inicio      = millis();
  tiempo_inicio_v    = tiempo_inicio;
  tiempo_inicio_giro = tiempo_inicio;
  pwm_der = 50;
  pwm_izq = 50;
  tiempo_por_girar = (theta / 360) * (cir_robot / velocidad_giro);
  #if debug
    Serial.print("Tiempo a girar: ");
    Serial.println(tiempo_por_girar);
  #endif
  ult_mov = mov_derecha;
}

void robot::movilidad::izquierda(double theta){
  tiempo_inicio      = millis();
  tiempo_inicio_v    = tiempo_inicio;
  tiempo_inicio_giro = tiempo_inicio;
  pwm_der = 50;
  pwm_izq = 50;
  tiempo_por_girar = (theta / 360) * (cir_robot / velocidad_giro);
  #if debug
    Serial.print("Tiempo a girar: ");
    Serial.println(tiempo_por_girar);
  #endif
  ult_mov = mov_izquierda;
}

void robot::movilidad::adelante(double distancia_recorrer){
  tiempo_inicio   = millis();
  tiempo_inicio_v = tiempo_inicio;
  this->distancia_recorrer = distancia_recorrer;
  ult_mov = mov_adelante;
}

void robot::movilidad::atras(double distancia_recorrer){
  tiempo_inicio   = millis();
  tiempo_inicio_v = tiempo_inicio;
  this->distancia_recorrer = distancia_recorrer;
  ult_mov = mov_atras;
}

void robot::movilidad::detener(){
  conteo_der = 0;
  conteo_izq = 0;
  v = 2;
  v1 = 2;
  last_error_d = 0;
  last_error_i = 0;
  distancia_anterior = 0;
  ult_mov = mov_detener;
  actMov();
}

void robot::movilidad::actMov(){
  int pwm_d = (ult_mov != mov_detener)?((ult_mov == mov_adelante || ult_mov == mov_izquierda)?pwm_der:-pwm_der):zero_pwm;
  int pwm_i = (ult_mov != mov_detener)?((ult_mov == mov_adelante || ult_mov == mov_derecha)?pwm_izq:-pwm_izq):zero_pwm;
  motor_derecho.setSpeed(pwm_d);
  motor_izquierdo.setSpeed(pwm_i);
}

int robot::movilidad::getPinEnDer(){
  return pin_en_derecho;
}
int robot::movilidad::getPinEnIzq(){
  return pin_en_izquierdo;
}
double robot::movilidad::getKpDer(){
  return Kp_d;
}
double robot::movilidad::getKdDer(){
  return Kd_d;
}
void robot::movilidad::setKpDer(double Kp){
  if(Kp > 0)
    this->Kp_d = Kp;
}
void robot::movilidad::setKdDer(double Kd){
  if(Kd > 0)
    this->Kd_d = Kd;
}
void robot::movilidad::setConstantesPdDer(double Kp, double Kd){
  setKpDer(Kp);
  setKdDer(Kd);
}
double robot::movilidad::getKpIzq(){
  return Kp_i;
}
double robot::movilidad::getKdIzq(){
  return Kd_i;
}
void robot::movilidad::setKpIzq(double Kp){
  if(Kp > 0)
    this->Kp_i = Kp;
}
void robot::movilidad::setKdIzq(double Kd){
  if(Kd > 0)
    this->Kd_i = Kd;
}
void robot::movilidad::setConstantesPdIzq(double Kp, double Kd){
  setKpIzq(Kp);
  setKdIzq(Kd);
}
inline void robot::movilidad::setVelocidad(int nueva_velocidad){
	velocidad_requerida = (nueva_velocidad >= min_velo && nueva_velocidad <= max_velo)?nueva_velocidad:velocidad_requerida;
}
