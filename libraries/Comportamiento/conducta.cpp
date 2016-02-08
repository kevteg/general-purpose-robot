#include "conducta.h"
robot::comportamiento::comportamiento(char nombre_robot,
                                      int pin_motor_d,
                                      int pin_motor_i,
                                      int velocidad_ini,
                                      int dist_sen_pin,
                                      int max_distan_sd,
                                      int pin_sir1,
                                      int pin_sir2,
                                      int pin_en_der,
                                      int pin_en_izq) :
                                      movimiento(pin_motor_d, pin_motor_i, pin_en_der, pin_en_izq, velocidad_ini),
                                      sensor_dist(dist_sen_pin, max_distan_sd),
                                      promedio_distancia(max_distan_sd),
                                      SENSOR_INFRAojo(pin_sir1, pin_sir2),
                                      nombre_robot(nombre_robot)
                                      {
  /*Tiempos para cambios en las rutinas de comportamientos básicos*/
  cambio_movimiento_eva = t_espera_max;
  cambio_rutina_vag = t_espera_min;
  /*Los pines de los leds estan uno al lado del otro*/
  for (int i = 0; i < numero_leds; i++)
      pinMode(led_verde + i, OUTPUT);
  encenderLed(ningun_led);

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

  /*Las excepeciones no estan activas hasta que el robot este calibrado*/
  if(robot_calibrado){
    if(activar_excepcion[SENSOR_DIST])
      distancia = promedio_distancia.add(sensor_dist.getDistance());
    if(activar_excepcion[SENSOR_INFRA]){
      posicion = SENSOR_INFRAojo.lectura(); //Posición de una linea relativo al sensor ir
      for(int i = 0; i < numero_sensores_ir; i++)
        lectura_sensores_ir[i] = SENSOR_INFRAojo.valorSensor(i);
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
        if(tiempo_transcurrido >= cambio_rutina_vag*seg || (_excepcion_sensor[SENSOR_DIST] || _excepcion_sensor[SENSOR_INFRA])){
          _vagar();
          tiempo_inicio = tiempo_actual;
        }
    break;
  	case e_rutina:
        if(_excepcion_sensor[SENSOR_DIST] || _excepcion_sensor[SENSOR_INFRA])
             cambiarMovimiento(e_detenido);
  	break;
    case e_calibrar:
      tiempo_actual = millis();
      tiempo_transcurrido = tiempo_actual - tiempo_inicio;
      if(tiempo_transcurrido >= tiempo_calibrar*seg){
          robot_calibrado = true;
          encenderLed(ningun_led);
          cambiarComportamiento(estado_ini_r);
      }
    break;
  }
}

void robot::comportamiento::encenderLed(int led_a_encender){
  switch (led_a_encender) {
    case todos_leds:
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_rojo, HIGH);
    break;
    case ningun_led:
      digitalWrite(led_verde, LOW);
      digitalWrite(led_rojo, LOW);
    break;
    default:
      digitalWrite(led_a_encender, HIGH);
      digitalWrite((led_a_encender == led_verde)?led_rojo:led_verde, LOW);
    break;
  }
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
          cambiarMovimiento(e_detenido);
          mens_correcto = true;
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
              default:
                mov = esperar;
              break;
            }

            mensa = (mov != esperar)?
            ((String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)seguir_i + (String)separador + (String)mov):
            ((String)nombre_robot + (String)separador + (String)buscar + (String)separador + (String)seguir_i);
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
              activar_excepcion[SENSOR_DIST] = (comando[7] == '1')?true:false;
            break;
            case excep_infra:
              mens_correcto = true;
              activar_excepcion[SENSOR_INFRA] = (comando[7] == '1')?true:false;
            break;
            default:
              enviarMensaje((String)nombre_robot +(String)separador + (String)error_comando);
            break;
          }
        }else
          enviarMensaje((String)nombre_robot + (String)separador + (String)error_excep + (String)separador);
      break;
      }
      case velocidad:{
        //<X:V:[0, 1]:[0, 9]>
        int opc_1, opc_2;
        opc_1 = comando[5] - 48;
        opc_2 = comando[7] - 48;
        if(opc_1 >= 0 && opc_1 <= 2 && opc_2 >= 0 && opc_2 <= 9)
          cambiarVel(opc_1, opc_2);
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
  if(activar_excepcion[SENSOR_DIST]){
      if(distancia > MUY_CERCA){
        _excepcion_sensor[SENSOR_DIST] = false;
        if(n_veces_exc[SENSOR_DIST] > 0){
          n_veces_exc[SENSOR_DIST]--;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_dist + (String)separador + (String)'0');
        }

      }else{
        _excepcion_sensor[SENSOR_DIST] = true;
        if(n_veces_exc[SENSOR_DIST] < num_envios_per){
          n_veces_exc[SENSOR_DIST]++;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_dist + (String)separador + (String)'1');
        }
      }
  }else
    _excepcion_sensor[SENSOR_DIST] = false;

  if(activar_excepcion[SENSOR_INFRA]){
      if(lectura_sensor_1 > umbral_lectura_ir || lectura_sensor_2 > umbral_lectura_ir){
        _excepcion_sensor[SENSOR_INFRA] = true;
        if(n_veces_exc[SENSOR_INFRA] < num_envios_per){
          n_veces_exc[SENSOR_INFRA]++;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_infra + (String)separador + (String)'1');
        }
      }else{
        _excepcion_sensor[SENSOR_INFRA] = false;
        if(n_veces_exc[SENSOR_INFRA] > 0){
          n_veces_exc[SENSOR_INFRA]--;
          enviarMensaje((String)nombre_robot + (String)separador + (String)set_excepcion + (String)separador + (String)excep_infra + (String)separador + (String)'0');
        }

      }
  }else
    _excepcion_sensor[SENSOR_INFRA] = false;

  if(_excepcion_sensor[SENSOR_INFRA] || _excepcion_sensor[SENSOR_DIST])
    encenderLed(led_rojo);
}

void robot::comportamiento::_vagar(){
  /*Vagar 1.0*/
  int nue_r;
  if(_excepcion_sensor[SENSOR_DIST] || _excepcion_sensor[SENSOR_INFRA])//Si el sensor de ultrasonido o el de distancia han generado una excepción
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
  if(nuevo_estado != e_rutina)
    for(int i = 0; i < numero_excepciones; i++)
      activar_excepcion[i] = true;

  switch(nuevo_estado){
  	case e_esperando:
  		cambiarMovimiento(e_detenido);
  		estado_robot = e_esperando;
      enviarComportamiento();
  	break;
  	case e_seguir:
  		estado_robot = e_seguir;
      enviarComportamiento();
  	break;
  	case e_evasion:
      tiempo_inicio = millis();
  		tipo_evasion = rand() % 3;
  		cambiarMovimiento(e_avanza);
  		estado_robot = e_evasion;
      enviarComportamiento();
  	break;
    case e_vagar:
      tiempo_inicio = millis();
      cambiarMovimiento(e_avanza);
      estado_robot = e_vagar;
      enviarComportamiento();
    break;
  	case e_rutina:
  		estado_robot = e_rutina;
      enviarComportamiento();
  	break;
    case e_calibrar:
      tiempo_inicio = millis();
      encenderLed(todos_leds);
      cambiarMovimiento(e_avanza);
      SENSOR_INFRAojo.calibrar();
      estado_robot = e_calibrar;
    break;
  }

}
void robot::comportamiento::_evasionObstaculos(){
  /*Evasión 3.0*/
if(estado_movimiento != e_detenido)
	switch(estado_movimiento){
		case e_vuelta_d: case e_vuelta_i: case e_atras:
			if(!_excepcion_sensor[SENSOR_DIST]){

				movimiento.adelante();
				estado_movimiento = e_avanza;
			}
		break;
		case e_avanza:
			if(_excepcion_sensor[SENSOR_DIST]){
        if(led_encendido == led_verde)
          encenderLed();
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
void robot::comportamiento::enviarComportamiento(){
  String comando = (String)delimitador_i + (String)nombre_robot + (String)separador + (String)buscar + (String)delimitador_f;
  ejecutarComando(comando);
}
void robot::comportamiento::cambiarVel(int opc_1, int opc_2){
  //Nueva velocidad va de 55 a 245
  int nueva_velocidad = velocidad_minima + (opc_2 + opc_1*10)*10;
  movimiento.setVelocidad(nueva_velocidad);
}
