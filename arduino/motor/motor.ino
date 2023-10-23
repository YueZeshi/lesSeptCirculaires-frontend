#include <MeMegaPi.h>
#define interruptPin 18    //port1的中断口
#define NE1 31                 //port1的比较口
long count=0;
int piscount=0;
unsigned long time;
unsigned long last_time;
MeMegaPiDCMotor motor1(PORT1B);   
uint8_t motorSpeed = 50;
void setup()
{
    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(NE1, INPUT);
    pinMode(10,OUTPUT);
    attachInterrupt(digitalPinToInterrupt(interruptPin), blink,RISING);   //设置中断函数，并设置上升沿中断
    Serial.begin(9600);   
    motor1.run(-motorSpeed); 
}
void loop()
{
  sweep();
  //piston(&piscount);
  //Serial.println(piscount);
/*  time =millis(); 
    if(time-last_time>1000)     //当大于1000ms时，打印脉冲数
    {
          Serial.println(count);
          //piston(count);
          //count++;
          last_time=time;
   }*/
}
void blink()
{
    if (digitalRead(NE1)>0)   //判断是正转还是反转
    count++;
    else
    count--;
}
void piston(int *count)
{
  if (*count-(*count/2)==*count/2)   //判断是正转还是反转
  digitalWrite(10,HIGH);
  else
  digitalWrite(10,LOW);
  delay(1000);
  *count=*count+1;
}

void sweep(){
  if(count>100){
    motor1.run(motorSpeed); 
  }
  if(count<-100){
    motor1.run(-motorSpeed);
  }
}