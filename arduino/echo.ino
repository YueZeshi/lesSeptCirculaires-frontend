/*
 * Du point du vue megaPI : 
Serial <=> PC (9600)
Serial2 <=> RPI (9600)
*/


void setup() {
  // initialize both serial ports:
  Serial2.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial2.available()) {
    int inByte = Serial2.read();
    Serial.write(inByte);
  }

  // lecture depuis 0 (console Arduino) pour envoi dans port 1
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial2.write(inByte);
  }
}
