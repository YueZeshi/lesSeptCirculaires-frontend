#define PWMB 12
#define NE2 18
#define NE1 31
#define BL1 34
#define BL2 35
long int count=0;
int t_ini,t_fin,dt = 10,ddt=5;
int V;
int last_err,c_set,c_cur,err,int_err,der_err;
void setup() {
  // put your setup code here, to run once:
  c_set=0;
  Serial2.begin(9600);
  Serial.begin(9600);
  pinMode(PWMB,OUTPUT);
  pinMode(NE2,INPUT_PULLUP);
  pinMode(NE1,INPUT);
  pinMode(BL1,OUTPUT);
  pinMode(BL2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(NE2),inter,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read command
  if(Serial2.available()){
    int note=Serial2.read(); 
    c_set = note*50;
  }
  // PID control
  last_err=err;
  c_cur=count;
  err = c_cur-c_set;
  int_err += err*0.001*dt;
  der_err = (err-last_err)*0.001/dt;
  V = PID(err,int_err,der_err);
  // time control
  // t_ini = millis()%30000;
  // while((millis()%30000-t_ini)<dt){
  V = constrain(V,-9.0,9.0);
  if(V>=0)
  {
    int PWM = V*255/12;
    rotation_cw(PWM);
    delay(dt);
  }
  else
  {
    int PWM = -V*255/12;
    rotation_ccw(PWM);
    delay(dt);
  }
  // }
}


void rotation_ccw(int PWM){
analogWrite(PWMB,PWM);
digitalWrite(BL2,LOW);
digitalWrite(BL1,HIGH);
}
void rotation_cw(int PWM){
analogWrite(PWMB,PWM);
digitalWrite(BL1,LOW);
digitalWrite(BL2,HIGH);
}
void stop(){
analogWrite(12,0);
}

void inter(){
  if(digitalRead(NE1)>0)
  count++;
  else count--;
}
double PID(int err,int int_err,int der_err){
  double kp,ki,kd;
  kp = 0.1;
  ki=0.01;
  kd=20;
  return kp*err+ki*int_err+kd*der_err;
}
// #include <MeMegaPi.h>
// // #include <FlexiTimer2.h>
// #define interruptPin 18    //interrupt port of port1
// #define NE1 31                 //comperation port of port1
// long count=0;
// uint8_t motorspeed = 30; //set a motor idle speed pulses/0.1secod
// int lastcount =0;
// int speed=0;    //speed pulses/0.1second
// MeMegaPiDCMotor motor1(PORT1B);  
// int motoroutput=0;  //power output calculated of motor
// #define pidDt 100 //the interval of time of the pid controller

// void setup() {
//   // FlexiTimer2::set(pidDt,1.0/1000,calculate); 
//   // FlexiTimer2::start(); 
//   Serial.begin(9600);   
//   pinMode(interruptPin, INPUT_PULLUP);
//   pinMode(NE1, INPUT);
//   pinMode(10,OUTPUT);
//   attachInterrupt(digitalPinToInterrupt(interruptPin), blink,RISING); 
// }

// void loop() {
//   //erial.println(motoroutput);
//   motor1.run(motoroutput);
// }

// float i=0;  //pid controller and its variables
// float d=0;
// float sumerror=0;
// float lasterror=0;
// #define i_lim 80
// float pid(float speedcommand,float speedmeasured,float kp,float ki,float kd){
//   float error=speedmeasured-speedcommand;
//   sumerror+=error*pidDt;
//   i=ki*sumerror;
//   if(i>i_lim) sumerror=i_lim;
//   if(i<-i_lim) sumerror=-i_lim;
//   d=kd*(lasterror-error)/pidDt;
//   lasterror=error;
//   return kp*error+i+d;
// }

// void calculate(){   //controlled by the timer's interruption, for fixed frequency calculation
//   speed=(count-lastcount)*100/pidDt;
//   lastcount=count;
//   int speedcmd=sweep();
//   motoroutput=pid(speedcmd,speed,1,0.02,20);// change pid parameter here
//   Serial.println(speed);
//   if (motoroutput>120) motoroutput=120;
//   if (motoroutput<-120) motoroutput=-120;
// }

// void blink()//reading encoder pulses
// {
//     if (digitalRead(NE1)>0)   //detect rotation direction
//     count++;
//     else
//     count--;
//     Serial.println(count);
// }

// int sweep_init=0;
// int direction_flag=0;
// int sweep(){ //motor idling
//   if(sweep_init==0){
//     sweep_init=1;
//     direction_flag=0;
//   }
//   else{
//     if(count>500){
//       direction_flag=1;
//     }
//     if(count<-500){
//       direction_flag=0;
//     }
//   }
//   if(direction_flag==0){
//     return motorspeed;
//   }
//   if(direction_flag==1){
//     return -motorspeed;
//   }
// }