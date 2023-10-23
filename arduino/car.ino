

void setup() {
  // put your setup code here, to run once:
  // gauche
  pinMode(8,OUTPUT);
  pinMode(36,OUTPUT);
  pinMode(37,OUTPUT);
  // droite
  pinMode(12,OUTPUT);
  pinMode(34,OUTPUT);
  pinMode(35,OUTPUT);
  // communication
  Serial.begin(9600);
  // capteur
  pinMode(A12,INPUT);
  double distance_ass = 0.1;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(getDistance());
  avance(10);
  delay(500);
  arriere(100);
  delay(500);
  stop();
}
double getDistance(){
  digitalWrite(A12,HIGH);
  delayMicroseconds(10);
  digitalWrite(A12,LOW);
  int duree = pulseIn(A12,HIGH);
  double distance_cur = duree*340e-6/2;
  return distance_cur;
}
void avance(int PWM){
analogWrite(8,PWM);
analogWrite(12,PWM);
digitalWrite(37,LOW);
digitalWrite(36,HIGH);
digitalWrite(35,LOW);
digitalWrite(34,HIGH);
}
void arriere(int PWM){
analogWrite(8,PWM);
analogWrite(12,PWM);
digitalWrite(36,LOW);
digitalWrite(37,HIGH);
digitalWrite(34,LOW);
digitalWrite(35,HIGH);
}
void stop(){
analogWrite(8,0);
analogWrite(12,0);
}