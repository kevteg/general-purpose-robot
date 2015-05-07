#include "qtr_control_sensor.h"
robot::qtr_sensor::qtr_sensor(int pin_s1, int pin_s2) : qtra((unsigned char[]) {pin_s1, pin_s2},  NUM_SENSORS){}
void robot::qtr_sensor::calibrar(){
	for(int i = 0; i < 200; i++){
		qtra.calibrate();
	}
}
unsigned int robot::qtr_sensor::lectura(){
	unsigned int position = qtra.readLine(sensor_values);
	return position;
}
 unsigned int robot::qtr_sensor::valorSensor(int index){
	return (index >= 0 && index < NUM_SENSORS)?(sensor_values[index]):(5000);
}

