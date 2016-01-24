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
#define max_velo 20					/*Máxima velocidad +/- maxVelo*/
#define min_velo 3					/*Minima velocidad +/- minVelo*/
#define zero_pwm 0				/*Velocidad 0 para detener el robot*/
#define c_movimiento 11.30973 //Esta constante es 2*pi*diamétro de las ruedas, para calculas el movimiento de cada rueda
#define tiempo_verificacion 100
#define altos_rueda 6
/*Estos datos se obtienen estudiando por separado cada sensor para verficar el tiempo que tarda en descargarse el capacitor*/
/*Nota importante: los valores pueden cambiar si las distancias varian. Hay que asegurarse que esten bien*/
#define min_v_blanco_en_der 10
#define max_v_blanco_en_der 50
#define min_v_negro_en_der 100
#define max_v_negro_en_der 500

#define min_v_blanco_en_izq 10
#define max_v_blanco_en_izq 65
#define min_v_negro_en_izq 100
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
			int pwm_der;
			int pwm_izq;
			double last_error_d, last_error_i;
			double tiempo, tiempo_inicio;
			double tiempo_v, tiempo_inicio_v;
			int velocidad_requerida;
			int velocidad_verdadera;
			int v, v1;
			int conteo_der, conteo_izq;
			float Kp;
			float Kd;
			float Ki;
			double integral_d, integral_i;
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
			 * @brief Segundo constructor de la clase.
			 * @param puertoDerecho, puerto izquierdo, velocidad.
			 *  	  Puerto de los motores en la tarjeta de motores.
			 *		  Velocidad de los motores.
			 *		  Entre 3 cm/s y 20 cm/s
			 */
			 movilidad(int pin_derecho, int pin_izquierdo, int pin_en_derecho, int pin_en_izquierdo, int velocidad = min_velo);
			 /**
 			 * @brief Conteo encoder derecho
 			 *        Se llama a esta librería cuando el pin del motor derecho cambia de 1 a 0,
			 *        lo cual significa que ha habido un cambio
 			 */
			 void conteo_en_der();
			 /**
 			 * @brief Conteo encoder izquierdo
 			 *        Se llama a esta librería cuando el pin del motor izquierdo cambia de 1 a 0,
			 *        lo cual significa que ha habido un cambio
 			 */
			 void conteo_en_izq();
			 /**
 			 * @brief Esta función incrementa los conteos de los motores cada vez que se pasa
			 * sobre un espacio blanco en el encoder
 			 */
			 void conteoRevoluciones();
			 /**
 			 * @brief Este método prepara los sensores para obtener una respuesta
			 * (que será capturada por los métodos conteo_en_izq y conteo_en_der)
 			 */
			 void activarSensores();
			 /**
 			 * @brief Este método cambia los pwm de los dos motores a lo que este en pwm_der y pwm_izq
 			 */
			 void actMov();
			 /**
 			 * @brief Se calcula la velocidad en cm/s de cada motor
			 * @return retorna las velocidades de ambos motores como un vector
 			 */
			 double* calculoVelocidad();
			 /**
 			 * @brief  Calcula el nuevo pwm para cualquier motor
			 * @return el nuevo pwm para el motor en turno
			 * @param pwm_act: pwm actual del motor que se vaya a analizar
			 *        vel_req: velocidad que se desea alcanzar
			 *        vel_act: velocidad actual real
			 *        last_error: error anterior del pid del motor en cuestión
			 *        integral: resultado del control integral anterior del motor
 			 */
			 int updatePid(int pwm_act, int vel_req, double vel_act, double* last_error, double *integral);
			 /**
 			 * @brief Aquí se lleva el tiempo para las verificaciones de los motores
			 * se censan las velocidades y se corrigen con updatePid
 			 */
			 void proceso();
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
			* @brief Pin encoder derecho
			* @return Se retorna el pin del encoder derecho
			*/
			int getPinEnDer();
			/**
			* @brief Pin encoder izquierdo
			* @return Se retorna el pin del encoder izquierda
			*/
			int getPinEnIzq();
			/**
			* @brief Constante proporcional
			* @return Se retorna la constante proporcional actual
			*/
			double getKp();
			/**
			* @brief Constante integral
			* @return Se retorna la constante integral actual
			*/
			double getKi();
			/**
			* @brief Constante derivativa
			* @return Se retorna la constante derivativa actual
			*/
			double getKd();
			/**
			* @brief cambiar constante proporcional
			*/
			void setKp(double Kp);
			/**
			* @brief cambiar constante integral
			*/
			void setKi(double Ki);
			/**
			* @brief cambiar constante derivativa
			*/
			void setKd(double Kd);
			/**
			* @brief cambiar todas las constantes del pid
			*/
			void setConstantesPid(double Kp, double Ki, double Kd);

			/**
			 * @brief Cambiar velocidad actual del robot
			 * @param nuevaVelocidad, nueva velocidad de los motores
			 */
			inline void setVelocidad(int nueva_velocidad);
	};
};

#endif
