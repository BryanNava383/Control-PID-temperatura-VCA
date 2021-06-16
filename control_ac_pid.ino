#include <avr/io.h>
#include <avr/interrupt.h>
#include <PID_v1.h> // se incluye la libreria PID

#define DETECT 2  //zero cross detect
#define GATE 9    //TRIAC gate
#define PULSE 4   //trigger pulse width (counts)
#define alpha 0.1 //valor alpha de la ecuacion para el filtro digital
int i=483;
//se definen las variables para poder usar el PID
double Setpoint, Input, Output;
//Se especifican las constantes para el PID (Kp= proporcional,Ki= integral, Kd= derivativo)
double Kp=20, Ki=3 , Kd=5;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//Variables para la lectura del LM35
int sensor;
int sensor_filtrado;
double ycontrol;
//Se declara una bandera para hacer sincronismo de la lectura de la temperatura y la ejecucion del codigo
int flag = 0;

void setup(){
   Serial.begin(9600);
  // set up pins
  pinMode(DETECT, INPUT);     //zero cross detect
  digitalWrite(DETECT, HIGH); //enable pull-up resistor
  pinMode(GATE, OUTPUT);      //TRIAC gate control

  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 100;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled


  // set up zero crossing interrupt
  attachInterrupt(0,zeroCrossingInterrupt, RISING);    
    //IRQ0 is pin 2. Call zeroCrossingInterrupt 
    //on rising signal
    //turn the PID on
    //Seteamos el Modo de PID en automatico para la correccion del error
  myPID.SetMode(AUTOMATIC);

}  

//Interrupt Service Routines

void zeroCrossingInterrupt(){ //zero cross detect  
//Se setea el valor de la bandera en 1 cuando la interrupcion es detectada
 flag = 1;
  TCCR1B=0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
}

ISR(TIMER1_COMPA_vect){ //comparator match
  digitalWrite(GATE,HIGH);  //set TRIAC gate to high
  TCNT1 = 65536-PULSE;      //trigger pulse width
}

ISR(TIMER1_OVF_vect){ //timer1 overflow
  digitalWrite(GATE,LOW); //turn off TRIAC gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}



void loop(){ // sample code to exercise the circuit
  //Si el valor de la bandera es 1 se procede a ejecutar el codigo
 if(flag == 1){
int pot= analogRead(A2);    
//Lectura y ajuste del potenciometro a los valores deseados
Setpoint= map(pot,0,1023,23, 50);
Serial.print(Setpoint);
Serial.print(" ");
/*
DEBIDO A QUE LA LECTURA DEL SENSOR LM35 GENERABA MUCHO RUIDO 
SE DECIDIO IMPLEMETAR UN FILTRO DIGITAL PASA BAJAS
 */
sensor=analogRead(A5);

sensor_filtrado= (alpha* sensor + ((1- alpha)*sensor_filtrado));
//Input= sensor;
Input = ((sensor_filtrado* 5000.0)/1023)/10; //lectura y ajuste del LM35 a los valores deseados

Serial.println(Input);
//Serial.print("   ");
myPID.Compute();
  if (Output >255) {Output = 255;} 
  if (Output < 0 ) {Output = 0;} 
//Serial.print(Output);
//Serial.print("   ");

ycontrol=map(Output,0,255,483,65);

//Serial.println(ycontrol);
OCR1A = (int)ycontrol;     //Se ajusta el tiempo de disparo del triac al valor del output del PID.
 //Se setea de nuevo el valor de la bandera en 1 
 //para poder ejecutar el codigo de nuevo hasta la siguiente deteccion de la interrupcion
 flag =  0;
}
}
