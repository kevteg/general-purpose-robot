#include <promedio.h>

#include <NewPing.h>
#include <control_sensor.h>
#include <newping_control_sensor.h>
#define echoPin 15
#define triggerPin 14
#define maxDistance 100
using namespace robot;
sensorUltra sensor(triggerPin, echoPin, maxDistance);
promedioDinamico<int, 3> promedio(maxDistance);
void setup(){
  Serial.begin(9600);
}
void loop(){
  Serial.println(promedio.add(sensor.getDistance()));
  delay(500);
}
