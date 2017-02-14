#include "infra_control_sensor.h"

robot::sensorInfra::sensorInfra(int pin) :
 controladorSensor(maxDist), pin(pin){}

robot::sensorInfra::sensorInfra(int pin, int max_distance) :
  controladorSensor(max_distance), pin(pin){}

unsigned int robot::sensorInfra::getDistance(){
	int distance = 4800/(analogRead(pin) - 20);
	return (distance <= 0)?max_distance:distance;
}
