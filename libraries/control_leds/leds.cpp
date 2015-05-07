#include "leds.h"
robot::control_leds::control_leds(int *puertos_leds, int n){
	this->n = n;
	this->puertos_leds = new int[n];
	estado_leds = new bool[n];
	
	for(int i = 0; i < n; i++)
		this->puertos_leds[i] = puertos_leds[i];
	for(int i = 0; i < n; i++){
		pinMode(this->puertos_leds[i], OUTPUT);
		estado_leds[i] = false;
		digitalWrite(this->puertos_leds[i], (estado_leds[i]?HIGH:LOW));
	}
}

void robot::control_leds::mostrarNumero(int i){
	int j = 3;
	int a = 0;
	int num = pow(2, n);
	char buffer[n];
	itoa(i, buffer, 2); 
	Serial.print("9 = ");
	Serial.println(puertos_leds[0]); //Para ver si funciona 
	while(!(i >= 0 && i <= num) && j < n + 3){
		estado_leds[a] = ((buffer[j++] == '1')?true:false);
		digitalWrite(puertos_leds[a], (estado_leds[a++]?HIGH:LOW));
	}
	
}

void robot::control_leds::combinacion(int *led_c, int n, int time){
	for(int i = 0; i < n; i++){
		estado_leds[i] = !estado_leds[i];
		digitalWrite(puertos_leds[led_c[i]], estado_leds[i]);
		delay(time);
		estado_leds[i] = !estado_leds[i];
		digitalWrite(puertos_leds[led_c[i]], estado_leds[i]);
	}
}