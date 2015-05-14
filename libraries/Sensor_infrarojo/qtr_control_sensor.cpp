#include "qtr_control_sensor.h"
robot::qtr_sensor::qtr_sensor(int pin_s1, int pin_s2) {

	/*unsigned char* arr = new unsigned char[2];
	arr[0] = '1';
	arr[1] = '2';*/
	/*char arr[2];
	arr[0] = pin_s1;
	arr[1] = pin_s2;*/

	//unsigned char arr[] = {'5', '5'};

  //_qtra((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},  NUM_SENSORS);

	// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively

}
void robot::qtr_sensor::calibrar(){
	for(int i = 0; i < 200; i++){
		_qtra.calibrate();
	}
}
unsigned int robot::qtr_sensor::lectura(){
	unsigned int position = _qtra.readLine(sensor_values);
	return position;
}
 unsigned int robot::qtr_sensor::valorSensor(int index){
	return (index >= 0 && index < NUM_SENSORS)?(sensor_values[index]):(5000);
}
