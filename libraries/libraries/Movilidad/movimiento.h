/**
 * @file movilidad.h
 * @brief Clase para controlar el movimiento de las ruedas
 * @author Kevin Hernández, Ángel Gil
 */
#ifndef ROBOT_MOVILIDAD_H_
#define ROBOT_MOVILIDAD_H_

#include <AFMotor.h> 				/*Controlador de la tarjeta adafruit*/
#include <adafruit_control_motor.h> /*Controlador del motor usando la librería de adafruit */
#include <control_motor.h> 			/*Definición de controlador del motor*/
#define maxVelo 255					/*Máxima velocidad +/- maxVelo*/
#define minVelo 55					/*Minima velocidad +/- minVelo*/
#define noVelocidad 0				/*Velocidad 0 para detener el robot*/
/*Nota: por defecto el robot comenzará moviéndose hacia adelante*/
namespace robot{
	class movilidad{
		private:
			Motor motor_derecho;
			Motor motor_izquierdo;
			int pin_derecho;
			int pin_izquierdo;
			int velocidad;
		public:
			/**
			 * @brief Primer constructor de la clase.
			 * @param puertoDerecho, puerto izquierdo.
			 *  	  Puerto de los motores en la tarjeta de motores.
			 *		  Valor por defecto de la velocidad maxVelo.
			 */
			 movilidad(int pin_derecho, int pin_izquierdo);
			/**
			 * @brief Segundo constructor de la clase.
			 * @param puertoDerecho, puerto izquierdo, velocidad.
			 *  	  Puerto de los motores en la tarjeta de motores.
			 *		  Velocidad de los motores.
			 *		  Entre 155 y 255.
			 */
			 movilidad(int pin_derecho, int pin_izquierdo, int velocidad);
			/**
			 * @brief Dirección: derecha.
			 * 		  Motor derecho hacia atrás y motor izquierdo hacia adelante
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo 
			 *		  Nota: el robot de mantiene girando hasta recibir otra orden
			 */
			void derecha();
			/**
			 * @brief Dirección: izquierda.
			 * 		  Motor derecho hacia adelante y motor izquierdo hacia atrás
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo 
			 *		  Nota: el robot de mantiene girando hasta recibir otra orden
			 */
			void izquierda();
			/**
			 * @brief Dirección: adelante.
			 * 		  Motor derecho hacia adelante y motor izquierdo hacia adelante
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo 
			 *		  Nota: el robot de mantiene yendo al frente hasta recibir otra orden
			 */
			void adelante();
			/**
			 * @brief Dirección: atrás.
			 * 		  Motor derecho hacia atrás y motor izquierdo hacia atrás
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo 
			 *		  Nota: el robot de mantiene yendo atrás hasta recibir otra orden
			 */
			void atras();
			/**
			 * @brief Robot detenido.
			 * 		  Motor derecho e izquierdo a velocidad 0
			 *		  Nota: el robot de mantiene detenido hasta recibir otra orden
			 */
			void detener();
			/**
			 * @brief Cambiar velocidad actual del robot
			 * @param nuevaVelocidad, nueva velocidad de los motores
			 */
			inline void setVelocidad(int nueva_velocidad);
	};
};

#endif