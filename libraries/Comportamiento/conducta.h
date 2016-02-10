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
#define MAX_DIST_SD               100  //Máxima distancia por defecto del sensor de distancia en cm
#define VELOCIDAD_MAXIMA          20  //Máxima velocidad por defecto de los motores
#define VELOCIDAD_MINIMA          5   //Velocidad minima
#define T_ESPERA_MIN              5    //Tiempo de espera minimo para cualquier rutina
#define T_ESPERA_MAX              20   //Tiempo de espera máximo para cualquier rutina
#define SEG                       1000 //1 segundo (1000 ms)
#define T_ENVIO                   0.5  //Tiempo de envio de mensajes, medio segundo cada mensaje
#define NUM_ENVIOS_PER            5    //Número de mensajes de excepeción que se envian cada vez que pasa la excepción
#define LED_VERDE                 18   //Puerto del led verde CAMBIAR CON LA NUEVA CONFIGURACIÓN <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define LED_ROJO                  19   //Puerto del led rojo
#define TODOS_LEDS                0    //Indica que todos los leds están encedidos
#define NINGUN_LED                -1   //Indica que no esta encendido ningún led, estado inicial
#define NUM_SENSORES_IR           2    //Número de sensores infrarojos del robot por defecto
#define NUM_LEDS                  2    //Número de leds del robot por defecto
#define SENSOR_IR_IZQ             0    //Posición del sensor ir de la izquierda en el vector de lecturas
#define SENSOR_IR_DER             1    //Posición del sensor ir de la derecha en el vector de lecturas
#define UMBRAL_LECTURA_IR         700  //Encima de este umbral se considera una línea negra
#define SENSOR_DIST               0    //Posición en el vector de excepciones de la excepción de sensor ultrasonido
#define SENSOR_INFRA              1    //Posición en el vector de excepciones de la excepción de sensor infrarojo
#define NUM_EXCEPCIONES           2    //Cantidad de excepciones que se tendrán
#define BAUDIOS                   9600 //Baudios de la comunicación serial
#define TODOS_ROBOTS              'X'  //Indica que la orden es para todos los robots
#define TIEMPO_CALIBRAR           3    //Tiempo para calibrarse
#define CVAGAR                    'A'  //Vagar del protócolo
#define CEVADIR                   'B'  //Evadir del protócolo
#define CSEGUIR_I                 'C'  //Seguir instrucciones del protócolo
#define CCALIBRA                  'D'  //Calibrando en el protócolo
#define CESPERAR                  '!'  //Poner al robot en espera del protócolo
#define CBUSCAR                   '?'  //Se pregunta por el robot, el robot envia un mensaje con lo que hace
#define DELIMITADOR_I             '<'  //Delimitador inicial de mensajes
#define DELIMITADOR_F             '>'  //Delimitaador final de mensajes
#define SEPARADOR                 ':'  //Separador de los mensajes
#define EX_DIST                   '0'  //Mensaje de error en caso de generarse una excepción de distancia
#define EX_INFRA                  '1'  //Mensaje de error en caso de generarse una excepción de sensores
#define ERROR_COMANDO             '_'  //Comando no reconocido (el robot lo envia)
#define CEXCEPCION                'E'  //Para apagar las excepciones
#define ERROR_EX                  '/'  //Sucede cuando el robot no puede apagar la excepcion (el robot lo envia)
#define MENSAJE_RECIBIDO          '#'  //El robot lo envia cuando ha recibido correctamente una orden
#define CVELOCIDAD                'V'  //Velocidad del robot
#define BUFFER_MENSAJES           10   //Cantidad máxima de elementos que pueden llegar como comando

 namespace robot{
    class comportamiento{
		private:
			movilidad movimiento;
			sensorUltra sensor_dist;
			qtr_sensor sensor_infrarojo;
			promedioDinamico <int, 3> promedio_distancia;
      bool en_comando;
      int indice_c;
      String *comando;
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
			 *	    max_distan_us: Máxima distancia a medir por el sensor de ultrasonido
			 *		  pin_sir1: Pin del sensor infrarojo 1
			 *      pin_sir2: Pin del sensor infrarojo 2
       *      pin_en_der: Pin encoder derecho
       *      ṕin_en_izq: Pin encoder izquierdo
			 */
			comportamiento(char nombre_robot, int pin_motor_d, int pin_motor_i, int velocidad_ini = velocidad_minima, int dist_sen_pin, int max_distan_sd = max_distancia_sd, int pin_sir1, int pin_sir2, int pin_en_der, int pin_en_izq);

      /**
      * @brief Interpretar los comandos que se reciben para cambiar el estado del robot
      */
      void ejecutarComando(String comando);
      /**
      * @brief Interpretar los comandos que se reciben para cambiar el estado del robot
      */
      void ejecutarComando(String *comando, int tam);
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
      void encenderLed(int led_a_encender);
      /**
       * @brief Cambia la velocidad a la nueva que llega al robot
       *@param opc_1 primera opción de cambio de velocidad
       *       opc_2 segunda opción de cambio de velocidad
       */
      void cambiarVel(int opc_1, int opc_2);
      /**
       * @brief Envia que comportamiento tiene el robot en el momento, se usa cuando el robot cambia su comportamiento
       *        Ej: Explorar->Esperar
       */
      void enviarComportamiento();

      /**
      * @brief actualizar el estado del robot según la lectura del sensor de ultrasonido o según lo que reciba por el xBee
      * 		  debe ser llamado repetidamente
      */
      void run();

    };
};
#endif
