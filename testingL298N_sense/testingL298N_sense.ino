int motorFL_speed = 3;            //front left enA
int motorFL_1 = 4;                //front left in1
int motorFL_2 = 5;                //front left in2
int motorBL_speed = 6;            //back left enB
int motorBL_1 = 7;                //back left in3
int motorBL_2 = 8;                //back left in4

void setup() {
  // put your setup code here, to run once:
  pinMode(motorFL_speed, OUTPUT);
  pinMode(motorFL_1, OUTPUT);
  pinMode(motorFL_2, OUTPUT); 
  pinMode(motorBL_speed, OUTPUT);
  pinMode(motorBL_1, OUTPUT);
  pinMode(motorBL_2, OUTPUT); 
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(motorFL_1, HIGH);
  digitalWrite(motorFL_2, LOW);
  analogWrite(motorFL_speed, 255);
  
  digitalWrite(motorBL_1, HIGH);
  digitalWrite(motorBL_2, LOW);
  analogWrite(motorBL_speed, 255);

  Serial.println(analogRead(A0)/1024.0*5.0/31.7);
}
