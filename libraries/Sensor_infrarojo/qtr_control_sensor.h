/**
 * @file qtr_control_sensor.h
 * @brief Librería para controlar dos sensores infrarojos
 * @author Kevin Hernández, Ángel Gil
 */

#ifndef ROBOT_QTR_SENSOR_H_
#define ROBOT_QTR_SENSOR_H_
#define NUM_SENSORS 2

#include <QTRSensors.h>
namespace robot{
	class qtr_sensor{
		private:
			QTRSensorsAnalog *_qtra;
			unsigned int sensor_values[NUM_SENSORS];

		public:
			/**
			 * @brief Primer constructor de la clase.
			 * @param pin_s1, puerto analógico para el sensor 1
			 *		  pin_s2, puerto analógico para el sensor 2
			 */
			qtr_sensor(int pin_s1, int pin_s2);
			/**
			 * @brief Calibrar los dos sensores para ser mas precisos con las mediciones que obtendran
			 */
			void calibrar();
			/**
			 * @brief Lectura de los sensores
			 * @return La posible posición de la linea sobre algún sensor
			 */
			unsigned int lectura();
			/**
			 * @brief Valor de los sensores, por defecto valores mas cercanos a cero
   			 * significan superficies blancas o reflectantes y al acercarse a 1000 negras u obscuras
			 * @return La lectura del sensor en la posición index, si no existe el sensor index se retorna un numero mayor a mil
			 */
			  unsigned int valorSensor(int index);
		};
};
#endif
