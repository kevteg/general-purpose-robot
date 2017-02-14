/**
 * @file newping_control_sensor.h
 * @brief Implementación del controlador del sensor de ultrasonido HC-SR04, usando la libreria newping.
 * @author Kevin Hernández, Ángel Gil
 */

#ifndef INFRA_CONTROL_SENSOR_H_
#define INFRA_CONTROL_SENSOR_H_

#include "control_sensor.h"

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif

#define maxDist 200 /*Si no se indica la distacia máxima sera 200*/

namespace robot{
    class sensorInfra : controladorSensor{
	private:
        int pin;
    public:
		/**
		 * @brief Primer constructor de la clase.
		 * @param pin
		 */
        sensorInfra(int pin);
    /**
		 * @brief Segundo constructor de la clase.
		 * @param pin, max_distance
		 */
        sensorInfra(int pin, int max_distance);
		/**
		 * @brief Método que retorna distancia actual que percibe el sensor.
		 * @return distancia actual.
		 */
        unsigned int getDistance();
    };
};

#endif
