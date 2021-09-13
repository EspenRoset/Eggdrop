#include <AVR_RTC.h>



//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/ 
//**************************************

#include <util/atomic.h>

//**************************************

#define ENCA 2      //Encoder pinA
#define ENCB 3      //Encoder pinB
#define PWM 11       //motor PWM pin
#define IN2 6       //motor controller pin2
#define IN1 7       //motor controller pin1

volatile int posi = 0; // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/ 
long prevT = 0;
float eprev = 0;
float eintegral = 0;
bool firstrun = true;
unsigned int timeout =0;
const int dir = -1;

//-20cm = -8657

//********************************************

void setup() {

  Serial.begin (230400);
  
  // ENCODER
  pinMode (ENCA, INPUT);
  pinMode (ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

//************************************************

void loop() {

 if(millis()>100){
  
  //time diference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/(1.0e6);
  prevT = currT;
   
  if (firstrun){
    firstrun=false;
    timeout = millis()+500;
  }

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
  }


 float u;
    if (millis()>timeout){
    u=0;
    float pwr = fabs(u);  //fabs == floating point, absolute value
    //Motor direction
    int dir = 1; 
    //Send signal to motor
   setMotor(dir,pwr,PWM,IN1,IN2);
    
  }
  else{
    u = -225;
  
   
  //Motor power
  float pwr = fabs(u);  //fabs == floating point, absolute value

  //Send signal to motor
 setMotor(dir,pwr,PWM,IN1,IN2);
 Serial.print(u);
 Serial.print(",");
 Serial.print(pos);
 Serial.print(",");
 Serial.print(micros());
 Serial.println();
  }
 }
 else{
   Serial.print("0.00,0,");
   Serial.println(micros());
 }
}

//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

//MOTOR
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


//ENCODER
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
