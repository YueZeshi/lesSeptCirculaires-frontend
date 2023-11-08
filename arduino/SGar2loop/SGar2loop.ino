#define freq 100
const int Dt = 1000000/100;//set loop frequency
#define PWMB 12
#define NE2 18
#define NE1 31
#define BL1 34
#define BL2 35
#define EM 10
float Vcount=0;//rotation speed, pulse/s
int counttime=0;
int lastcounttime=0;//time between counts, to calculate rotation speed
int looptime=0;
int lastlooptime=0;//measuring loop time for further delay
long count=0;//position measured by encoder, by pulses
unsigned long du_reponse,t_command;
int tc=200,tc_pp=40;
int V;
double c_set,c_cur;
int tape_command,configuration_mode;
int note;
void setup() {
  c_set=0;
  du_reponse=600;//à déterminer grâce au temps de réponse maximal de moteur
  tape_command=0;
  configuration_mode=1;
  Serial2.begin(9600);
  Serial.begin(9600);
  pinMode(PWMB,OUTPUT);
  pinMode(NE2,INPUT_PULLUP);
  pinMode(NE1,INPUT);
  pinMode(BL1,OUTPUT);
  pinMode(BL2,OUTPUT);
  pinMode(EM,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(NE2),inter,RISING);
  digitalWrite(EM,LOW);
}

void loop() {
  // read command
  lastlooptime=looptime;
  if(Serial2.available()){
    note=Serial2.read(); 
    switch(note){
      case 0:
      case 20:
      case 30:
      case 21:
      case 31:
        configuration_mode=1;
        break;
      default:
        configuration_mode=0;
        c_set = (note-1)*50;
        tape_command=1;
        t_command=millis();
        break;
    }
  }
  if(configuration_mode){
    switch(note){
      case 20:
        rotation(5);
        delay(tc);
        break;
      case 21:
        rotation(2);
        delay(tc_pp);
        break;
      case 30:
        rotation(-5);
        delay(tc);
        break;
      case 31:
        rotation(-2);
        delay(tc_pp);
        break;
      default:
        stop();    
    }
    note=0;
    count=0;
  }
  else{
    int V_set=pida(c_set,count,1,0,0);//angle loop controller
    V=pidv(V_set,Vcount,1,0,0);//speed loop controller //double loop, set your pid parameters here
    V = constrain(V,-9.0,9.0);
    rotation(V);
    if(tape_command&&(millis()-t_command)>du_reponse){//abs(der_err)<2 &&abs(err)<5 && tape_fin==0){
      digitalWrite(EM,HIGH);
      delay(50);//30 mieux
      digitalWrite(EM,LOW);
      tape_command=0;
    }
  }
  looptime=micros();
  delayMicroseconds(Dt-looptime+lastlooptime);
}



void rotation(int V){
  if(V>=0){
    int PWM = V*255/12;
    analogWrite(PWMB,PWM);
    digitalWrite(BL1,LOW);
    digitalWrite(BL2,HIGH);
  }
  else{
    int PWM = -V*255/12;
    analogWrite(PWMB,PWM);
    digitalWrite(BL2,LOW);
    digitalWrite(BL1,HIGH);
  }
}

void stop(){
analogWrite(12,0);
}

void inter(){
  counttime=micros();
  if(digitalRead(NE1)>0){
    Vcount=1000000/(counttime-lastcounttime);
    count++;
  }
  else{
    count--;
    Vcount=-1000000/(counttime-lastcounttime);
  }
  lastcounttime=counttime;
}

float i=0;  //pid controller of speed and its variables
float d=0;
float sumerror=0;
float lasterror=0;
#define i_lim 80
float pidv(float speedcommand,float speedmeasured,float kp,float ki,float kd){
  float error=speedmeasured-speedcommand;
  sumerror+=error*Dt;
  i=ki*sumerror;
  if(i>i_lim) sumerror=i_lim;
  if(i<-i_lim) sumerror=-i_lim;
  d=kd*(lasterror-error)/Dt;
  lasterror=error;
  return kp*error+i+d;
}

float ia=0;  //pid controller of angle and its variables
float da=0;
float sumerrora=0;
float lasterrora=0;
#define i_lima 80
float pida(float speedcommand,float speedmeasured,float kp,float ki,float kd){
  float error=speedmeasured-speedcommand;
  sumerror+=error*Dt;
  ia=ki*sumerror;
  if(ia>i_lima) sumerrora=i_lima;
  if(ia<-i_lima) sumerrora=-i_lima;
  da=kd*(lasterrora-error)/Dt;
  lasterrora=error;
  return kp*error+ia+da;
}
