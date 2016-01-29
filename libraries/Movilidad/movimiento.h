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
#define c_movimiento 11.30973 //Esta constante es 2*pi*radio de las ruedas, para calculas el movimiento de cada rueda
#define tiempo_verificacion 50
#define altos_rueda 6
#define distancia_reinicio 30 //Se reinicia el pid cada que pasa esta distancia en centimétros
#define cir_robot 24.5044227  //7.8 (L) * pi
#define velocidad_giro 5.0
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

#define debug true
#define reinicio_30_cm false
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
			double distancia_promedio;
			double distancia_promedio_total;
			double distancia_anterior;
			double distancia_recorrer;
			double distancia_d;
			double distancia_i;
			int velocidad_requerida;
			int velocidad_verdadera;
			int v, v1;
			int conteo_der, conteo_izq;
			double Kp_d, Kp_i;
			double Kd_d, Kd_i;
			double tiempo_giro, tiempo_inicio_giro;
			double tiempo_por_girar;
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
			 double calculoVelocidad(int mot);
			 /**
 			 * @brief  Calcula el nuevo pwm para cualquier motor
			 * @return el nuevo pwm para el motor en turno
			 * @param pwm_act: pwm actual del motor que se vaya a analizar
			 *        vel_req: velocidad que se desea alcanzar
			 *        vel_act: velocidad actual real
			 *        last_error: error anterior del pid del motor en cuestión
			 *        integral: resultado del control integral anterior del motor
 			 */
			 int updatePid(double Kp, double Kd, int pwm_act, int vel_req, double vel_act, double* last_error);
			 /**
 			 * @brief Aquí se lleva el tiempo para las verificaciones de los motores
			 * se censan las velocidades y se corrigen con updatePid
 			 */
			 void proceso();
			 void control(double velocidad_d, double velocidad_i, double distancia_promedio, double velocidad_requerida);
			/**
			 * @brief Dirección: derecha.
			 * 		  Motor derecho hacia atrás y motor izquierdo hacia adelante
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo
			 *
			 */
			void derecha(double theta);
			/**
			 * @brief Dirección: izquierda.
			 * 		  Motor derecho hacia adelante y motor izquierdo hacia atrás
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo
			 *
			 */
			void izquierda(double theta);
			/**
			 * @brief Dirección: adelante.
			 * 		  Motor derecho hacia adelante y motor izquierdo hacia adelante
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo
			 *		  Nota: el robot de mantiene yendo al frente hasta recibir otra orden
			 */
			void adelante(double distancia_recorrer);
			/**
			 * @brief Dirección: atrás.
			 * 		  Motor derecho hacia atrás y motor izquierdo hacia atrás
			 *		  Ambos motores a la misma velocidad, definida por el usuario o, por defecto maxVelo
			 *		  Nota: el robot de mantiene yendo atrás hasta recibir otra orden
			 */
			void atras(double distancia_recorrer);
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
			* @brief Constante proporcional motor derecho
			* @return Se retorna la constante proporcional actual
			*/
			double getKpDer();
			/**
			* @brief Constante derivativa motor derecho
			* @return Se retorna la constante derivativa actual
			*/
			double getKdDer();
			/**
			* @brief cambiar constante proporcional motor derecho
			*/
			void setKpDer(double Kp);

			/**
			* @brief cambiar constante derivativa motor derecho
			*/
			void setKdDer(double Kd);
			/**
			* @brief cambiar todas las constantes del pid motor derecho
			*/
			void setConstantesPdDer(double Kp, double Kd);
			/**
			* @brief Constante proporcional motor izquierdo
			* @return Se retorna la constante proporcional actual
			*/
			double getKpIzq();
			/**
			* @brief Constante derivativa motor izquierdo
			* @return Se retorna la constante derivativa actual
			*/
			double getKdIzq();
			/**
			* @brief cambiar constante proporcional motor izquierdo
			*/
			void setKpIzq(double Kp);
			/**
			* @brief cambiar constante derivativa motor izquierdo
			*/
			void setKdIzq(double Kd);
			/**
			* @brief cambiar todas las constantes del pid motor izquierdo
			*/
			void setConstantesPdIzq(double Kp, double Kd);

			/**
			 * @brief Cambiar velocidad actual del robot
			 * @param nuevaVelocidad, nueva velocidad de los motores
			 */
			inline void setVelocidad(int nueva_velocidad);
	};
};

#endif
