#include <MsTimer2.h> 
#include <TimerOne.h>
#include <AFMotor.h> 
#include <adafruit_control_motor.h> 
#include <control_motor.h>

#define LRSPIN 2
using namespace robot;
Motor _motor(4, 255);

volatile boolean chargeRS;
volatile unsigned long leftRSStartTime;     
volatile unsigned long leftRSTimer;
unsigned int n;
int v;
void chargeRSISR() {      
  chargeRS = true;      
}
void conteo(){
  Timer1.detachInterrupt();
  int rotacion = (n / 6);
  Serial.print(rotacion);
  Serial.println(" rotaciones por segundo");
  n = 0;
  Timer1.attachInterrupt(conteo);
}
void leftRSISR() {
  leftRSTimer = micros() - leftRSStartTime;      
}      

void setup() {      
  n = 0;
  v = 0;
  Serial.begin(57600);      
  Serial.println("Starting...");      
  
  attachInterrupt(digitalPinToInterrupt(2), leftRSISR, FALLING);      
  chargeRS = false;      
  Timer1.initialize(1000000);
  MsTimer2::set(2, chargeRSISR); // 2ms period
  MsTimer2::start();      
  Timer1.attachInterrupt(conteo);
}

void loop() {
  if (chargeRS) {      
    pinMode(LRSPIN, OUTPUT);    // make line an output      
    digitalWrite(LRSPIN, HIGH); // drive line high       
  
    delayMicroseconds(10);      // charge line      
    
    pinMode(LRSPIN, INPUT);     // make line an input      
    leftRSStartTime = micros();            
    chargeRS = false;      
  }      
  if (leftRSTimer) {      

    if(leftRSTimer > 39 && leftRSTimer < 61){
      if(v == 2)
        n++;
      v = 1;
    }else if(leftRSTimer > 199 && leftRSTimer < 401)
      v = 2;
    leftRSTimer = 0;      
  }      
}
