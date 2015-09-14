#include "adafruit_control_motor.h"


robot::Motor::Motor(int number, int actual_speed) : controladorMotor(), motor(number){
	setSpeed(actual_speed);
}

robot::Motor::Motor(int number) : controladorMotor(), motor(number), actual_speed(0){}

void robot::Motor::setSpeed(int speed){
	actual_speed = speed;
	if (speed >= 0) {
		motor.setSpeed(speed);
		motor.run(FORWARD);
	}else{
		motor.setSpeed(-speed);
		motor.run(BACKWARD);
	}
}

inline int robot::Motor::getSpeed(){ 
	return actual_speed;
}

