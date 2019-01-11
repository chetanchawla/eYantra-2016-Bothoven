void setup() {
pinMode(22,OUTPUT);
pinMode(23,OUTPUT);
pinMode(24,OUTPUT);
pinMode(25,OUTPUT);//to give o/p the motors
pinMode(45,OUTPUT);
pinMode(46,OUTPUT);//to give o/p to both the l293ds
}

void loop() {
analogWrite(45,128);
analogWrite(46,128);
digitalWrite(22,HIGH);
digitalWrite(23,LOW);
digitalWrite(24,LOW);
digitalWrite(25,HIGH);
delay(2000);
digitalWrite(22,LOW);
digitalWrite(23,HIGH);
digitalWrite(24,HIGH);
digitalWrite(25,LOW);
delay(2000);

}
