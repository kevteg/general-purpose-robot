#include "conducta.h"
robot::comportamiento::comportamiento(char nombre_robot, int pin_motor_d, int pin_motor_i, int velocidad_ini, int trigger_pin, int echo_pin, int max_distan_us, int pin_sir1, int pin_sir2) :
movimiento(pin_motor_d, pin_motor_i, velocidad_ini), sensor_ultra(trigger_pin, echo_pin, max_distan_us),
promedio_distancia(max_distan_us), sensor_infrarojo(pin_sir1, pin_sir2), nombre_robot(nombre_robot){
  movimiento.detener();
  cambio_movimiento_eva = t_espera_max;
  cambio_rutina_vag = t_espera_min;
  for (int i = 0; i < numero_leds; i++)
      pinMode(led_verde + i, OUTPUT);
  /*Para indicar que se esta calibrando se encienden ambos leds*/
  digitalWrite(led_verde, LOW);
  digitalWrite(led_rojo, LOW);
  led_encendido = ningun_led;
  n_veces_exc       = new int[numero_excepciones];
  _excepcion_sensor = new bool[numero_excepciones];
  activar_excepcion = new bool[numero_excepciones];
  for(int i = 0; i < numero_excepciones; i++){
    n_veces_exc[i] = 0;
    _excepcion_sensor[i] = false;
    activar_excepcion[i] = true; //Por defecto las excepciones están encendidas
  }
  robot_calibrado = false;
  estado_ini_r = e_esperando; //Por defecto al menos que se indique lo contrario al iniciar el robot
}

robot::comportamiento::comportamiento(char nombre_robot, int pin_motor_d, int pin_motor_i, int trigger_pin, int echo_pin, int pin_sir1, int pin_sir2) :
movimiento(pin_motor_d, pin_motor_i, velocidad_defecto_motores), sensor_ultra(trigger_pin, echo_pin, max_distancia_ultrasonido),
promedio_distancia(max_distancia_ultrasonido), sensor_infrarojo(pin_sir1, pin_sir2), nombre_robot(nombre_robot){
  movimiento.detener();
  cambio_movimiento_eva = t_espera_max;
  cambio_rutina_vag = t_espera_min;
  for (int i = 0; i < numero_leds; i++)
    pinMode(led_verde + i, OUTPUT);
  /*Para indicar que se esta calibrando se encienden ambos leds*/
  digitalWrite(led_verde, LOW);
  digitalWrite(led_rojo, LOW);
  led_encendido = ningun_led;
  n_veces_exc       = new int[numero_excepciones];
  _excepcion_sensor = new bool[numero_excepciones];
  activar_excepcion = new bool[numero_excepciones];
  for(int i = 0; i < numero_excepciones; i++){
    n_veces_exc[i] = 0;
    _excepcion_sensor[i] = false;
    activar_excepcion[i] = true; //Por defecto las excepciones están encendidas
  }
  robot_calibrado = false;
  estado_ini_r = e_esperando; //Por defecto al menos que se indique lo contrario al iniciar el robot
}

void robot::comportamiento::inicializar(){
  cambiarComportamiento(e_calibrar);
  Serial.begin(baudios);
}

void robot::comportamiento::run(){
  unsigned long tiempo_actual;
  unsigned int lectura_sensores_ir[numero_sensores_ir];
  unsigned long tiempo_transcurrido;
  int distancia;
  unsigned int posicion;

  //Las excepeciones no estan activas hasta que el robot este calibrado
  if(robot_calibrado){
    if(activar_excepcion[sensor_ultras])
      distancia = promedio_distancia.add(sensor_ultra.getDistance());
    if(activar_excepcion[sensor_infrar]){
      posicion = sensor_infrarojo.lectura(); //Posición de una linea relativo al sensor ir
      for(int i = 0; i < numero_sensores_ir; i++)
        lectura_sensores_ir[i] = sensor_infrarojo.valorSensor(i);
      /*Serial.println(lectura_sensores_ir[0]);
      Serial.println(lectura_sensores_ir[1]); */
    }
    excepcion(distancia, lectura_sensores_ir[sensor_ir_izq], lectura_sensores_ir[sensor_ir_der]);
  }
  escuchar();

  switch(estado_robot){
  		case e_evasion:
        tiempo_actual = millis();
        _evasionObstaculos();
  			if(tiempo_transcurrido >= cambio_movimiento_eva*seg){
  				tipo_evasion = rand() % 3;
          cambio_movimiento_eva = (!tipo_evasion)?t_espera_min:t_espera_max;
  				tiempo_inicio = tiempo_actual;
  			}
  		break;
  	case e_seguir:
      //No definido
		break;
    case e_vagar:
        tiempo_actual = millis();
        tiempo_transcurrido = tiempo_actual - tiempo_inicio;
        if(tiempo_transcurrido >= cambio_rutina_vag*seg || (_excepcion_sensor[sensor_ultras] || _excepcion_sensor[sensor_infrar])){
          _vagar();
          tiempo_inicio = tiempo_actual;
        }
    break;
  	case e_rutina:
        if(_excepcion_sensor[sensor_ultras] || _excepcion_sensor[sensor_infrar]){
             cambiarMovimiento(e_detenido);
             if(led_encendido == led_verde)
               cambioLed();
        }
  	break;
    case e_calibrar:
      tiempo_actual = millis();
      tiempo_transcurrido = tiempo_actual - tiempo_inicio;
      if(tiempo_transcurrido >= tiempo_calibrar*seg){
          robot_calibrado = true;
          cambiarComportamiento(estado_ini_r);
      }
    break;
  }
}
void robot::comportamiento::cambioLed(){
  if(led_encendido != ningun_led){
    led_encendido = (led_encendido == led_rojo || led_encendido == todos_leds)?led_verde:led_rojo;
    digitalWrite(led_encendido, HIGH);
    digitalWrite((led_encendido == led_verde)?led_rojo:led_verde, LOW);
  }else{
    led_encendido = todos_leds;
    digitalWrite(led_verde, HIGH);
    digitalWrite(led_rojo, HIGH);
  }

}
void robot::comportamiento::_seguirLinea(unsigned int lectura_sensor_1, unsigned int lectura_sensor_2, unsigned int posicion_linea){}

void robot::comportamiento::cambiarMovimiento(estado_m rutina){
  estado_movimiento = rutina;
    switch(rutina){
      case e_avanza:
          movimiento.adelante();
      break;
      case e_atras:
          movimiento.atras();
      break;
      case e_vuelta_d:
          movimiento.derecha();
      break;
      case e_vuelta_i:
          movimiento.izquierda();
      break;
      case e_detenido:
          movimiento.detener();
      break;
    }
}
bool robot::comportamiento::_realizarRutina(char rutina){
  bool corr = true;
  switch(rutina){
    case '1':
      cambiarMovimiento(e_avanza);
      break;
    case '2':
      cambiarMovimiento(e_atras);
      break;
    case '3':
      cambiarMovimiento(e_vuelta_d);
      break;
    case '4':
      cambiarMovimiento(e_vuelta_i);
      break;
    case '5':
      cambiarMovimiento(e_detenido);
    break;
    default:
      corr = false;
    break;
  }
  return corr;
}

void robot::comportamiento::escuchar(){
  char c;
  bool salir = false;

  while(Serial.available() > 0 && !salir){
    c = Serial.read();
    if(comando[0] != delimitador_i){
      salir = true;
      comando = "";
    }
    comando += c;
  }
  if(comando.length() > 0 && comando[0] == delimitador_i && comando[comando.length() -1 ] == delimitador_f){
    ejecutarComando(comando);
    comando = "";
  }
}
void robot::comportamiento::ejecutarComando(String comando){
  bool mens_correcto = false;
  if(comando[1] == nombre_robot || comando[1] == todos_robot){
    if(!robot_calibrado)
      cambiarComportamiento(e_calibrar);

    switch(comando[3]){
      case vagar:{
        mens_correcto = true;
        if(estado_robot != e_vagar){
          if(!robot_calibrado)
            estado_ini_r = e_vagar;
          else
            cambiarComportamiento(e_vagar);
        }
         break;
      }
      case evadir:{
        mens_correcto = true;
        if(estado_robot != e_evasion){
          if(!robot_calibrado)
            estado_ini_r = e_evasion;
          else
            cambiarComportamiento(e_evasion);
        }
        break;
      }

      case seguir_i:{
        if(estado_robot != e_rutina){
          if(!robot_calibrado)
            estado_ini_r = e_rutina;
          else
            cambiarComportamiento(e_rutina);
        }
        if(comando[4] == separador )
          mens_correcto = _realizarRutina(comando[5]);
        else
          mens_correcto = true;
          cambiarMovimiento(e_detenido);
        break;
      }
      case esperar:{
        mens_correcto = true;
        if(estado_robot == e_rutina){
          cambiarMovimiento(e_detenido);

        }else if(estado_robot != e_esperando){
          if(!robot_calibrado)
            estado_ini_r = e_esperando;
            else
              cambiarComportamiento(e_esperando);
        }
        break;
      }
      case buscar:{
        mens_correcto = true;
        String mensa;
        switch(estado_robot){
          case e_esperando:
            mensa = (String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)esperar;
          break;
          case e_evasion:
            mensa = (String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)evadir;
          break;
          case e_vagar:
            mensa = (String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)vagar;
          break;
          case e_rutina:{
            char mov;
            switch(estado_movimiento){
              case e_avanza:
                mov = '1';
              break;
              case e_atras:
                mov = '2';
              break;
              case e_vuelta_d:
                mov = '3';
              break;
              case e_vuelta_i:
                mov = '4';
              break;
              case e_detenido:
                mov = esperar;
              break;
            }
            mensa = (String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)seguir_i + (String)mov;
          break;
          }
          case e_calibrar:
            mensa = (String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)calibra;
          break;
        }
        enviarMensaje(mensa);
        break;
      }
      case set_excepcion:{
        if(estado_robot == e_rutina){
          switch(comando[5]){
            case excep_dist:
              mens_correcto = true;
              activar_excepcion[sensor_ultras] = (comando[7] == '1')?true:false;
            break;
            case excep_infra:
              mens_correcto = true;
              activar_excepcion[sensor_infrar] = (comando[7] == '1')?true:false;
            break;
            default:
              enviarMensaje((String)nombre_robot +(String)separador + (String)error_comando);
            break;
          }
        }else
          enviarMensaje((String)nombre_robot + (String)separador + (String)error_excep + (String)separador);
      break;
      }

    }
    /*Por ahora no habran confirmaciones*/
    /*Si el comando esta mal deberia indicarse*/

    /*if(mens_correcto){
      if(comando[3] != buscar) // ya en buscar se envio una confirmacion
        enviarMensaje((String)nombre_robot + (String)separador + (String)mensaje_recibido);
    }else
      enviarMensaje((String)nombre_robot + (String)separador + (String)error_comando);
    */
  }
}

void  robot::comportamiento::enviarMensaje(String mensaje){
  String envio = delimitador_i + mensaje + delimitador_f;
  Serial.print(envio);
}
void robot::comportamiento::excepcion(int distancia, unsigned int lectura_sensor_1, unsigned int lectura_sensor_2){
  if(activar_excepcion[sensor_ultras]){
      if(distancia > MUY_CERCA){
        _excepcion_sensor[sensor_ultras] = false;
        if(n_veces_exc[sensor_ultras] > 0){
          n_veces_exc[sensor_ultras]--;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_dist + (String)separador + (String)'0');
          if(led_encendido == led_rojo && !_excepcion_sensor[sensor_infrar])
            cambioLed();
        }

      }else{
        _excepcion_sensor[sensor_ultras] = true;
        if(n_veces_exc[sensor_ultras] < num_envios_per){
          n_veces_exc[sensor_ultras]++;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_dist + (String)separador + (String)'1');
          if(led_encendido == led_verde)
            cambioLed();
        }


      }
  }else
    _excepcion_sensor[sensor_ultras] = false;

  if(activar_excepcion[sensor_infrar]){
      if(lectura_sensor_1 > umbral_lectura_ir|| lectura_sensor_2 > umbral_lectura_ir){
        _excepcion_sensor[sensor_infrar] = true;
        if(n_veces_exc[sensor_infrar] < num_envios_per){
          n_veces_exc[sensor_infrar]++;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_infra + (String)separador + (String)'1');
          if(led_encendido == led_verde)
            cambioLed();
        }


      }else if(lectura_sensor_1 <= umbral_lectura_ir && lectura_sensor_2 <= umbral_lectura_ir){
        _excepcion_sensor[sensor_infrar] = false;
        if(n_veces_exc[sensor_infrar] > 0){
          n_veces_exc[sensor_infrar]--;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_infra + (String)separador + (String)'0');
          if(led_encendido == led_rojo && !_excepcion_sensor[sensor_ultras])
            cambioLed();
        }

      }
  }else
    _excepcion_sensor[sensor_infrar] = false;

  if((!activar_excepcion[sensor_ultras] || !activar_excepcion[sensor_infrar]) && (led_encendido == led_rojo))
    cambioLed();
}
void robot::comportamiento::_vagar(){
  /*Vagar 1.0*/
  int nue_r;
  if(_excepcion_sensor[sensor_ultras] || _excepcion_sensor[sensor_infrar])//Si el sensor de ultrasonido o el de distancia han generado una excepción
  /*Se envia un mensaje para ver que se vio y se pone en modo espera*/
  cambiarComportamiento(e_esperando);
  else{
    switch(estado_movimiento){
      case e_avanza: case e_atras:
        nue_r = rand() % 2;
        cambio_rutina_vag = 1 + rand() % 1; //De 2 a 4 segundos estará girando
        if(nue_r == 0){
          estado_movimiento = e_vuelta_d;
          movimiento.derecha();
        }else{
          estado_movimiento = e_vuelta_i;
          movimiento.izquierda();
        }
      break;
      case e_vuelta_d: case e_vuelta_i:
         estado_movimiento = e_avanza;
         movimiento.adelante();
         cambio_rutina_vag = 5 + rand() % 2; //De 5 a 10 segundos hacia adelante
      break;
    }
  }
}

void robot::comportamiento::cambiarComportamiento(estado_r nuevo_estado){
  if(led_encendido == led_rojo || led_encendido == todos_leds || (led_encendido == ningun_led && robot_calibrado) || nuevo_estado == e_calibrar)
    cambioLed();
  if(nuevo_estado != e_rutina)
    for(int i = 0; i < numero_excepciones; i++)
      activar_excepcion[i] = true;

  switch(nuevo_estado){
  	case e_esperando:
  		cambiarMovimiento(e_detenido);
  		estado_robot = e_esperando;
  	break;
  	case e_seguir:
  		estado_robot = e_seguir;
  	break;
  	case e_evasion:
      tiempo_inicio = millis();
  		tipo_evasion = rand() % 3;
  		cambiarMovimiento(e_avanza);
  		estado_robot = e_evasion;
  	break;
    case e_vagar:
      tiempo_inicio = millis();
      cambiarMovimiento(e_avanza);
      estado_robot = e_vagar;
    break;
  	case e_rutina:
  		estado_robot = e_rutina;
  	break;
    case e_calibrar:
      tiempo_inicio = millis();
      cambiarMovimiento(e_avanza);
      sensor_infrarojo.calibrar();
      estado_robot = e_calibrar;
    break;
  }
}
void robot::comportamiento::_evasionObstaculos(){
  /*Evasión 3.0*/
if(estado_movimiento != e_detenido)
	switch(estado_movimiento){
		case e_vuelta_d: case e_vuelta_i: case e_atras:
			if(!_excepcion_sensor[sensor_ultras]){

				movimiento.adelante();
				estado_movimiento = e_avanza;
			}
		break;
		case e_avanza:
			if(_excepcion_sensor[sensor_ultras]){
        if(led_encendido == led_verde)
          cambioLed();
					switch(tipo_evasion){
						case 0:
							movimiento.atras();
							estado_movimiento = e_atras;
							tipo_evasion = 0;
						break;
						case 1:
							movimiento.derecha();
							estado_movimiento = e_vuelta_d;
							tipo_evasion = 1;
						break;
						case 2:
							movimiento.izquierda();
							estado_movimiento = e_vuelta_i;
							tipo_evasion = 2;
						break;
					}
			}
		break;
	}
}
