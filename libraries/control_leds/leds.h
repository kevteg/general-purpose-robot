/**
 * @file leds.h
 * @brief Clase para control de los leds
 * @author Kevin Hernández, Ángel Gil
 */
#ifndef _LEDS_CONTROL_H
#define _LEDS_CONTROL_H
#include <Arduino.h>
#include <math.h>
namespace robot{
	class control_leds{
		private:
			int n;
			int *puertos_leds;
			bool *estado_leds;
		public:
			/**
			 * @brief Primer constructor de la clase.
			 * @param 
			 */
			control_leds(int *puertos_leds, int n);
			/**
			 * @brief Mostrar el número i
			 * @param i: número decimal
			 */
			void mostrarNumero(int i);
			/**
			 * @brief Mostrar combinación de luces
			 * @param led_c: Vector de leds
			 *		  n: Número de leds
			 *	      time: tiempo en microsegundos por secuencia de leds
			 */
			void combinacion(int *led_c, int n, int time);
	};
};

#endif