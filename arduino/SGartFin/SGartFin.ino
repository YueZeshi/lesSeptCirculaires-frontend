#define PWMB 12
#define NE2 18
#define NE1 31
#define BL1 34
#define BL2 35
#define EM 10
long count=0;
unsigned long du_reponse,t_command;
int dt = 10;
int tc=200,tc_pp=20;
int V;
double c_set,c_cur,marge;
double last_err,err,int_err,der_err;
int tape_command,configuration_mode;
int note,last_note,diff_note;
double kp,ki,kd;
double a1,a3;
double m,m0;
int note2count[]={0,50,102,152,200,250,300,350,400,450,500,547};
void setup() {
  // put your setup code here, to run once:
  kp = 0.2;//0.8
  ki=0.02;//0.02
  kd=100;//100;
  err=0;
  last_note=1;
  diff_note=0;
  m = 2.5;//-2
  m0 = 1;
  c_set=0;
  du_reponse=850;//à déterminer grâce au temps de réponse maximal de moteur
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
  // put your main code here, to run repeatedly:
  // read command
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
        diff_note=note-last_note;
        if(diff_note>0)marge=+m;
        else marge=-m;
        if(diff_note==0)marge=0;
        c_set = -note2count[note-1]+marge;
        tape_command=1;
        last_note=note;
        t_command=millis();
        break;
    }
  }
  if(configuration_mode){
    int PWM = 5*255/12;
    int PWM_pp = 2*255/12;
    switch(note){
      case 20:
        rotation_cw(PWM);
        delay(tc);
        count=m0;
        break;
      case 21:
        rotation_cw(PWM_pp);
        delay(tc_pp);
        count=m0;
        break;
      case 30:
        rotation_ccw(PWM);
        delay(tc);
        count=-m0;
        break;
      case 31:
        rotation_ccw(PWM_pp);
        delay(tc_pp);
        count=-m0;
        break;
      default:
        stop();
        break;    
    }
    note=0;
    err=0;
    last_err=0;
    int_err=0;
    der_err=0;
  }
  else{
    // PID control
    last_err=err;
    c_cur=count;
    err = c_cur-c_set;
    int_err += err*0.001*dt;
    der_err = (err-last_err)*0.001/dt;
    //*0.001/dt;
    V = PID(err,int_err,der_err);
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
    if(tape_command&&(millis()-t_command)>du_reponse){//abs(der_err)<2 &&abs(err)<5 && tape_fin==0){
      digitalWrite(EM,HIGH);
      delay(50);//30 mieux
      digitalWrite(EM,LOW);
      tape_command=0;
    }
    
  }
  //Serial.println(V);
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
  return kp*err+ki*int_err+kd*der_err;
}
