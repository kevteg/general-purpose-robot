/**
 * @file comportamiento.h
 * @brief Librería de control del comportamiento del robot
 * @author Kevin Hernández, Ángel Gil
 */

#ifndef ROBOT_CONDUCTA_H_
#define ROBOT_CONDUCTA_H_
/*Movilidad*/
#include <AFMotor.h>
#include <adafruit_control_motor.h>
#include <control_motor.h>
#include <movimiento.h>
/*Sensores*/
#include <NewPing.h>
#include <QTRSensors.h>
#include <control_sensor.h>
#include <newping_control_sensor.h>
#include <qtr_control_sensor.h>
/*Comunicación*/
/*Auxiliares*/
#include <promedio.h>
/*Definiciones*/
#define MUY_CERCA                 15   //Distancia en cm
#define max_distancia_ultrasonido 100  //Máxima distancia por defecto del ultrasonido en cm
#define velocidad_defecto_motores 100  //Máxima velocidad por defecto de los motores
#define t_espera_min              5    //Tiempo de espera minimo para cualquier rutina
#define t_espera_max              20   //Tiempo de espera máximo para cualquier rutina
#define seg                       1000 //1 segundo (1000 ms)
#define t_envio                   0.5  //Tiempo de envio de mensajes, medio segundo cada mensaje
#define num_envios_per            5    //Número de mensajes de excepeción que se envian cada vez que pasa la excepción
#define led_verde                 18   //Puerto del led verde
#define led_rojo                  19   //Puerto del led rojo
#define todos_leds                0    //Indica que todos los leds están encedidos
#define ningun_led                -1   //Indica que no esta encendido ningún led, estado inicial
#define numero_sensores_ir        2    //Número de sensores infrarojos del robot por defecto
#define numero_leds               2    //Número de leds del robot por defecto
#define sensor_ir_izq             0    //Posición del sensor ir de la izquierda en el vector de lecturas
#define sensor_ir_der             1    //Posición del sensor ir de la derecha en el vector de lecturas
#define umbral_lectura_ir         700  //Encima de este umbral se considera una línea negra
#define sensor_ultras             0    //Posición en el vector de excepciones de la excepción de sensor ultrasonido
#define sensor_infrar             1    //Posición en el vector de excepciones de la excepción de sensor infrarojo
#define numero_excepciones        2    //Cantidad de excepciones que se tendrán
#define baudios                   9600 //Baudios de la comunicación serial
#define todos_robot               'X'  //Indica que la orden es para todos los robots
#define tiempo_calibrar           3    //Tiempo para calibrarse
#define vagar                     'A'  //Vagar del protócolo
#define evadir                    'B'  //Evadir del protócolo
#define seguir_i                  'C'  //Seguir instrucciones del protócolo
#define calibra                   'D'  //Calibrando en el protócolo
#define esperar                   '!'  //Poner al robot en espera del protócolo
#define buscar                    '?'  //Se pregunta por el robot, el robot envia un mensaje con lo que hace
#define delimitador_i             '<'  //Delimitador inicial de mensajes
#define delimitador_f             '>'  //Delimitaador final de mensajes
#define separador                 ':'  //Separador de los mensajes
#define excep_dist                '0'  //Mensaje de error en caso de generarse una excepción de distancia
#define excep_infra               '1'  //Mensaje de error en caso de generarse una excepción de sensores
#define error_comando             '_'  //Comando no reconocido (el robot lo envia)
#define set_excepcion             'E'  //Para apagar las excepciones
#define error_excep               '/'  //Sucede cuando el robot no puede apagar la excepcion (el robot lo envia)
#define mensaje_recibido          '#'  //El robot lo envia cuando ha recibido correctamente una orden

 namespace robot{
    class comportamiento{
		private:
			movilidad movimiento;
			sensorUltra sensor_ultra;
			qtr_sensor sensor_infrarojo;
			promedioDinamico <int, 3> promedio_distancia;
      char nombre_robot;
			enum estado_m{
						  e_vuelta_d,
						  e_vuelta_i,
						  e_avanza,
						  e_detenido,
						  e_atras
						  };
			enum estado_r{
						  e_evasion,//Evasión de obstaculos: solo avanza y evita obstaculos
						  e_seguir, //Seguir una linea
              /*
              * El robot explora aleatoriamente el espacio hasta que consigue algún
              * objeto o una linea, en ese momento su estado cambia a esperando y avisa
              */
              e_vagar,
              /*
              * El robot esta esperando ordenes,
              * ejecutará la misma orden hasta que llegue otra
              */
						  e_rutina,
              /*
              * El robot espera ordenes:
              * sólo se da en el caso en que el robot este ejecuntado instrucciones
              * o espere a conseguir un objeto u otro robot (vagar)
              * no se usa cuando evade, sigue líneas.
              * las condiciones para que espere las da el estado actual
              */
						  e_esperando,
              e_calibrar
						  };
			int tipo_evasion;
      int led_encendido;              //Led encendido actualmente
      int cambio_movimiento_eva;      //Tiempo en el cual se cambia el movimiento en la rutina de evadir obstaculos
			int cambio_rutina_vag;          //Tiempo en el cual se cambia el movimiento en la rutina de vagar
      int *n_veces_exc;               //Número de veces seguidas que ha se ha enviado la excepción
      estado_m estado_movimiento;
			estado_r estado_robot;
      bool *_excepcion_sensor;        //Excepciones
			unsigned long tiempo_inicio;    //Tiempo de inicio para verificar las rutinas
      String comando;                 //Comandos que llegan al robor
			unsigned long tiempo_finalizar; //Tiempo en el cual se ejecutará una rutina
      bool robot_calibrado;           //Indica si el robot esta calibrado
      estado_r estado_ini_r;          //El estado en el que iniciará el robot
      bool *activar_excepcion;          //Usado para apagar excepciones
		public:
			/**
			 * @brief Primer constructor de la clase.
			 * @param pin_motor_d: Puerto a conectar el motor derecho
			 *		  pin_motor_i: Puerto a conectar el motor izquierdo
			 *		  velocidad_ini: Velocidad inicial de los motores
			 *  	  trigger_pin: Puerto trigger del sensor de ultrasonido
			 *		  echo_pin: Puerto echo del sensor de ultrasonido
			 *	      max_distan_us: Máxima distancia a medir por el sensor de ultrasonido
			 *		  pin_sir1: Pin del sensor infrarojo 1
			 *        pin_sir2: Pin del sensor infrarojo 2
			 */
			comportamiento(char nombre_robot, int pin_motor_d, int pin_motor_i, int velocidad_ini, int trigger_pin, int echo_pin, int max_distan_us, int pin_sir1, int pin_sir2);
			/**
			 * @brief Cuarto constructor de la clase.
			 * @param pin_motor_d: Puerto a conectar el motor derecho
			 *		  pin_motor_i: Puerto a conectar el motor izquierdo
			 *		  trigger_pin: Puerto trigger del sensor de ultrasonido
			 *		  echo_pin: Puerto echo del sensor de ultrasonido
			 *		  pin_sir1: Pin del sensor infrarojo 1
			 *        pin_sir2: Pin del sensor infrarojo 2
			 *		  Nota: la velocidad inicial si no se indica será velocidad_defecto_motores.
			 *		  Nota: la máxima distancia del sensor de ultrasonido será por defecto 200 cm.
			 */
			comportamiento(char nombre_robot, int pin_motor_d, int pin_motor_i, int trigger_pin, int echo_pin, int pin_sir1, int pin_sir2);
      /**
      * @brief Interpretar los comandos que se reciben para cambiar el estado del robot
      */
      void ejecutarComando(String comando);
      /**
      * @brief Generar movimiento aleatorio para evitar obstaculos
      */
      void _evasionObstaculos();
			/**
			* @brief Seguir una linea usando los sensores infrarojos, es necesario calibrar sensores antes
			* @param lectura_sensor_1 lectura del sensor infrarojo 1
			*	     lectura_sensor_2 lectura del sensor infrarojo 2
			*        posicion_linea posición relativa de la linea según las últimas lecturas
			*/
			void _seguirLinea(unsigned int lectura_sensor_1, unsigned int lectura_sensor_2, unsigned int posicion_linea);
			/**
			* @brief Cambiar movimiento actual al que se indique
			* @param rutina: nuevo movimiento del robot
			*/
			void cambiarMovimiento(estado_m rutina);
      /**
      * @brief Cambia la rutina actual usando el comando recibido
      * @param rutina: nueva rutina
      * @return true si es una rutina correcta
      */
      bool _realizarRutina(char rutina);
      /**
      * @brief El robot vaga aleatoriamente hasta que consigue algún objeto o
      *        se encuentra sobre una linea
      */
      void _vagar();
      /**
			 * @brief Escuchar ordenes del xbee, es en esta función donde se decide cambiar el estado del robot
			 */
			void escuchar();
      /**
      * @brief Enviar mensaje al controlador del robot
      * @param mensaje: mensaje a enviar a la computadora
      */
      void enviarMensaje(String mensaje);
			/**
			 * @brief Manejar errores durante la ejecución de alguna rutine (i.e encontrar un obstaculo mientras ejecuta una tarea)
       *        enciende o apaga las excepciones a las cual el robot respone según sea su comportamiento actual
			 *        También se envia un mensaje de error a la computadora
       *@param distancia: Distancia actual del robot con respecto al objeto mas próximo
       *        lectura_sensor_1: Lectura del sensor infrarojo 1
       *        lectura_sensor_2: Lectura del sensor infrarojo 2
       */
			void excepcion(int distancia, unsigned int lectura_sensor_1, unsigned int lectura_sensor_2);
			/**
			 * @brief Inicializar el estado del robot
			 */
			void inicializar();
			/**
			 * @brief Cambiar comportamiento del robot
			 */
			void cambiarComportamiento(estado_r nuevo_estado);
      /**
      * @brief Cambiar el estado de los leds, apaga el led que este encendido y enciende el otro
      */
      void cambioLed();
      /**
      * @brief actualizar el estado del robot según la lectura del sensor de ultrasonido o según lo que reciba por el xBee
      * 		  debe ser llamado repetidamente
      */
      void run();

    };
};
#endif
