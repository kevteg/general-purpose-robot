
#include <AFMotor.h>
#include <adafruit_control_motor.h>
#include <control_motor.h>
#define maxVelo 255
#define puertoMotor 1

using namespace robot;

Motor motor1(puertoMotor, maxVelo);
char lectura[20] = "";
int velocidadActual = maxVelo;

void setup(){
  Serial.begin(9600);
}

void loop(){
  if(Serial.available() > 0){
    Serial.readBytes(lectura, 20);
    decide(lectura);
    motor1.setSpeed(velocidadActual);  
    Serial.println(velocidadActual);
    strcpy(lectura, "      ");
    Serial.flush();
  }
}

void decide(char lectura[20]){
  if(!strcmp(lectura, "det"))
    velocidadActual = 0;
  else if(!strcmp(lectura, "ini"))
    velocidadActual = maxVelo;
}

