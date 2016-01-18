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
#include <Arduino.h>
#define maxVelo 255					/*Máxima velocidad +/- maxVelo*/
#define minVelo 55					/*Minima velocidad +/- minVelo*/
#define noVelocidad 0				/*Velocidad 0 para detener el robot*/
#define c_movimiento 11.30973 //Esta constante es 2*pi*diamétro de las ruedas, para calculas el movimiento de cada rueda
/*Estos datos se obtienen estudiando por separado cada sensor para verficar el tiempo que tarda en descargarse el capacitor*/
/*Nota importante: los valores pueden cambiar si las distancias varian. Hay que asegurarse que esten bien*/
#define min_v_blanco_en_der 5
#define max_v_blanco_en_der 100
#define min_v_negro_en_der 200
#define max_v_negro_en_der 2000

#define min_v_blanco_en_izq 39
#define max_v_blanco_en_izq 65
#define min_v_negro_en_izq 199
#define max_v_negro_en_izq 501
/*Nota: por defecto el robot comenzará moviéndose hacia adelante*/
namespace robot{
	class movilidad{
		private:
			Motor motor_derecho;
			Motor motor_izquierdo;
			volatile unsigned long tiempo_ini_en_der;
			volatile unsigned long tiempo_en_der;

			volatile unsigned long tiempo_ini_en_izq;
			volatile unsigned long tiempo_en_izq;
			int pin_derecho;
			int pin_izquierdo;
			int pin_en_derecho;
			int pin_en_izquierdo;
			int velocidad;
			int vel_der;
			int vel_izq;
			double tiempo, tiempo_inicio;
			double tiempo_v, tiempo_inicio_v;
			bool v, v1;
			int n, n1;
			enum movimientos{
							mov_adelante,
							mov_atras,
							mov_derecha,
							mov_izquierda,
							mov_detener
			};
			movimientos ult_mov;
		public:
			/**
			 * @brief Primer constructor de la clase.
			 * @param puertoDerecho, puerto izquierdo.
			 *  	  Puerto de los motores en la tarjeta de motores.
			 *		  Valor por defecto de la velocidad maxVelo.
			 */
			 movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo);
			/**
			 * @brief Segundo constructor de la clase.
			 * @param puertoDerecho, puerto izquierdo, velocidad.
			 *  	  Puerto de los motores en la tarjeta de motores.
			 *		  Velocidad de los motores.
			 *		  Entre 155 y 255.
			 */
			 movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo, int velocidad);
			 void conteo_en_der();
			 void conteo_en_izq();
			 void activarInterrupcion();
			 void verificarVelocidades();
			 void activarSensores();
			 void rectificacion(double v_r_der, double v_r_izq);
			 void actMov();
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
			int getPinEnDer();
			int getPinEnIzq();
			/**
			 * @brief Cambiar velocidad actual del robot
			 * @param nuevaVelocidad, nueva velocidad de los motores
			 */
			inline void setVelocidad(int nueva_velocidad);
	};
};

#endif
