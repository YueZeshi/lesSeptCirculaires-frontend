#include <MeMegaPi.h>
#include <FlexiTimer2.h>
MeMegaPiDCMotor motor1(PORT1B);  
MeMegaPiDCMotor motor2(PORT2B); 
float distance_measured=0;   
float distance = 0.15;
int motoroutput=0;
#define pidDt 100

void setup() {
  FlexiTimer2::set(pidDt,1.0/1000,calculate); 
  FlexiTimer2::start(); 
  Serial.begin(9600);   
}

void loop() {
  motor1.run(-motoroutput);
  motor2.run(motoroutput);
  Serial.println(distance_measured);
}

float i=0;
float d=0;
float sumerror=0;
float lasterror=0;
#define i_lim 50
int pid(float command,float measured,int kp,int ki,int kd,int dt){
  float error=measured-command;
  sumerror+=error*dt;
  if(sumerror>i_lim) sumerror=i_lim;
  if(sumerror<-i_lim) sumerror=-i_lim;
  i=ki*sumerror;
  d=kd*(lasterror-error)/dt;
  lasterror=error;
  return kp*error+i+d;
}

void calculate(){
  distance_measured=getDistance();
  motoroutput=pid(distance,distance_measured,1200,2,2000,pidDt);
  if (motoroutput>120) motoroutput=120;
  if (motoroutput<-120) motoroutput=-120;
}

double getDistance(){
  digitalWrite(A12,HIGH);
  delayMicroseconds(10);
  digitalWrite(A12,LOW);
  int duree = pulseIn(A12,HIGH);
  double distance_cur = duree*340e-6/2;
  return distance_cur;
}