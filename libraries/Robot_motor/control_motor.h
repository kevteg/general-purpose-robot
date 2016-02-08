/**
 * @file control_motor.h
 * @brief Definición básica del controlador del motor del robot.
 * @author Kevin Hernández, Ángel Gil
 */
#ifndef ROBOT_CONTROL_MOTOR_H_
#define ROBOT_CONTROL_MOTOR_H_

		namespace robot{

			class controladorMotor{
				public:
					/**
					 * @brief Cambiar velocidad del motor.
					 * @param speed, la nueva velocidad del robot.
					 *  Valores correctos entre -255 y 255 (Inclusive)
					 *  Velocidad > 0, motor va hacia adelante.
					 *  Velocidad < 0, motor va hacia atras.
					 *  Velocidad = 0, detiene el motor
					 */
					virtual void setSpeed(int speed) = 0;

					/**
					 * @brief Retornar la velidad actual del robot.
					 * @return La velocidad actual en un rango de -255 a 255.
					 */
					virtual int getSpeed() = 0;            
			};
			
		};
		
#endif