#include <promedio.h>

#include <NewPing.h>
#include <control_sensor.h>
#include <infra_control_sensor.h>
#include <SoftwareSerial.h>
#define pin 1
#define maxDistance 200
using namespace robot;
SoftwareSerial portTwo(9, 10);
sensorInfra sensor(pin);
promedioDinamico<int, 3> promedio(maxDistance);
void setup(){
  Serial.begin(9600);
  portTwo.begin(9600);
}
void loop(){
  portTwo.println(promedio.add(sensor.getDistance()));
  delay(500);
}
