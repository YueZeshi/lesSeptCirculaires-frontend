#include <MeMegaPi.h>
const int desiredFrequency = 50;  // 目标频率（以毫秒为单位，比如1秒一次）
unsigned long previousMillis = 0;
unsigned long interval = 1000 / desiredFrequency;  // 计算目标频率对应的间隔时间
#define interruptPin 18    //port1的中断口
#define NE1 31                 //port1的比较口
long count=0;
int piscount=0;
unsigned long time;
unsigned long last_time;
MeMegaPiDCMotor motor1(PORT1B);   
uint8_t motorSpeed = 5;
int speedcount =0;
int speed=0;

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(NE1, INPUT);
  pinMode(10,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink,RISING);   //设置中断函数，并设置上升沿中断
  Serial.begin(9600);   
}

void loop() {

  unsigned long currentMillis = millis(); // 记录当前时间
  if (currentMillis - previousMillis >= interval) { // 检查是否到达了下一个周期
    // 保存当前时间作为下一个周期的起始时间
    previousMillis = currentMillis;
    speed=(count-speedcount)*10/interval;
    speedcount=count;
    int motoroutput=pid(sweep(),speed,50,10,0);
    motor1.run(motoroutput);
    Serial.println(motoroutput);
  }
}

float i=0;
float d=0;
float sumerror=0;
float lasterror=0;
int pid(int speedcommand,int speedmeasured,int kp,int ki,int kd){
  int error=speedmeasured-speedcommand;
  sumerror+=error;
  i=ki*sumerror;
  d=kd*(error-lasterror)/interval;
  lasterror=error;
  return kp*error+i+d;
}


void blink()
{
    if (digitalRead(NE1)>0)   //判断是正转还是反转
    count++;
    else
    count--;
}

int sweep(){
  if(count>50){
    return motorspeed;
  }
  if(count<-50){
    return -motorSpeed;
  }
}