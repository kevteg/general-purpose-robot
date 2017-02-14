#include <MsTimer2.h> 
#include <TimerOne.h>
#include <AFMotor.h> 
#include <adafruit_control_motor.h> 
#include <control_motor.h>

#define pin_s_ir 2
using namespace robot;
Motor _motor(4, 255);

volatile boolean cargar_c;
volatile unsigned long t_inicio_des;     
volatile unsigned long t_des;
unsigned int n;
int v;
void activar_carga() {      
  cargar_c = true;      
}
void conteo(){
  Timer1.detachInterrupt();
  int rotacion = (n / 6);
  Serial.print(rotacion);
  Serial.println(" rotaciones por segundo");
  n = 0;
  Timer1.attachInterrupt(conteo);
}
void t_descarga() {
  t_des = micros() - t_inicio_des;      
}      
void act_sensor(){
    pinMode(pin_s_ir, OUTPUT);    // Se hace el pin del sensor un output      
    digitalWrite(pin_s_ir, HIGH); // Se escribe un alto para que el capacitor del sensor comience a cargarse       
  
    delayMicroseconds(10);      // Se esperan 10ms a que se cargue      
    
    pinMode(pin_s_ir, INPUT);     // Se hace input de nuevo para calcular el tiempo de descarga    
    t_inicio_des = micros();      //Tiempo de inicio de descarga      
    cargar_c = false;    
}
void setup() {      
  n = 0;
  v = 0;
  Serial.begin(57600);           
  attachInterrupt(digitalPinToInterrupt(2), t_descarga, FALLING); //Cuando el pin del sensor vaya de HIGH a LOW se habra descargado el capacitor
  cargar_c = false;      
  Timer1.initialize(1000000);
  MsTimer2::set(2, activar_carga); // Cada 2ms se hace proceso para revisar información del sensor
  MsTimer2::start();      
  Timer1.attachInterrupt(conteo);
}

void loop() {
  if (cargar_c)       
    act_sensor();
       
  if (t_des) {      
    if(t_des > 39 && t_des < 61){
      if(v == 2)
        n++;
      v = 1;
    }else if(t_des > 199 && t_des < 401)
      v = 2;
    t_des = 0;      
  }      
}
