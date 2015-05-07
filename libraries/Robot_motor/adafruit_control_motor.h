/**
 * @file adafruit_control_motor.h
 * @brief Controlador del motor usando la librería de adafruit.
 * @author Kevin Hernández, Ángel Gil
 */
#ifndef  ROBOT_ADAFRUIT_CONTROL_MOTOR_H
#define ROBOT_ADAFRUIT_CONTROL_MOTOR_H
/*
*Nota, lo pongo aquí (AFMotor) porque el compilador es un poco primitivo y si solo llamo esa libreria 
*en un lugar (aqui o en el .ino) el cpp no reconoce al objeto de tipo AF_DCMotor
*/
#include <AFMotor.h>
#include <control_motor.h>

	namespace robot{

		class Motor : public controladorMotor{
			private:
				AF_DCMotor motor;
				int actual_speed;
			public:
				/**
				 * @brief Segundo constructor de la clase.
				 * @param número del motor a controlar de 1 a 4, velocidad inicial (-255 a 255).
				 */
				Motor(int number, int actual_speed);
				/**
				 * @brief Tercer constructor de la clase.
				 * @param número del motor a controlar de 1 a 4.
				 */
				Motor(int number);
				/**
				 * @brief Cambiar la velocidad actual del motor
				 * @param speed: nueva velocidad
				 */
				void setSpeed(int speed);
				/**
				 * @brief Obtener la velocidad actual.
				 * @return velocidad actual
				 */
				inline int getSpeed();
		};
		
	};
	
#endif