void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
//pinMode(2,OUTPUT);
//pinMode(3,OUTPUT);
//
//digitalWrite(2,HIGH);
//digitalWrite(3,LOW);
pinMode(A0,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(analogRead(A0));
  Serial.println(map(analogRead(A0),0,1023,0,5000)/1000.0);
}
