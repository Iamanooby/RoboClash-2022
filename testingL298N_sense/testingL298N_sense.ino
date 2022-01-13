const int motorFL_speed = 6;            //front left enA
const int motorFL_1 = 29;                //front left in1
const int motorFL_2 = 27;                //front left in2
const int motorBL_speed = 7;            //back left enB
const int motorBL_1 = 25;                //back left in3
const int motorBL_2 = 23;                //back left in4


const int motorFR_speed = 9;            //front right enA
const int motorFR_1 = 31;                //front right in1
const int motorFR_2 = 33;                //front right in2
const int motorBR_speed = 8;           //back right enB
const int motorBR_1 = 35;               //back right in3
const int motorBR_2 = 37;               //back right in4

const int motorIL_speed = 10;            //intake left enA, unused
const int motorIL_1 = 43;                //intake left in1
const int motorIL_2 = 45;                //intake left in2
const int motorIR_speed = 11;           //intake right enB, unused
const int motorIR_1 = 39;               //intake right in3
const int motorIR_2 = 41;               //intake right in4

void setup() {
  // put your setup code here, to run once:
  pinMode(motorFL_speed, OUTPUT);
  pinMode(motorFL_1, OUTPUT);
  pinMode(motorFL_2, OUTPUT); 
  pinMode(motorFR_speed, OUTPUT);
  pinMode(motorFR_1, OUTPUT);
  pinMode(motorFR_2, OUTPUT); 
  pinMode(motorBL_speed, OUTPUT);
  pinMode(motorBL_1, OUTPUT);
  pinMode(motorBL_2, OUTPUT); 
  pinMode(motorBR_speed, OUTPUT);
  pinMode(motorBR_1, OUTPUT);
  pinMode(motorBR_2, OUTPUT); 

  pinMode(13,OUTPUT);

  Serial.begin(115200);
}

void loop() {
//  // put your main code here, to run repeatedly:
//  digitalWrite(13,HIGH);
//
//  digitalWrite(motorIL_1, HIGH);
//  digitalWrite(motorIL_2, HIGH);
//  analogWrite(motorIL_speed, 255);
//  
//  digitalWrite(motorIR_1, HIGH);
//  digitalWrite(motorIR_2, HIGH);
//  analogWrite(motorIR_speed, 255);
//
//

//strafe inwards by default with low power to manatain current draw

int stall_speed = 40;

  digitalWrite(motorFL_1, HIGH);
  digitalWrite(motorFL_2, LOW);
  analogWrite(motorFL_speed, stall_speed);
//  
  digitalWrite(motorFR_1, HIGH);
  digitalWrite(motorFR_2, LOW);
  analogWrite(motorFR_speed, stall_speed);
//
//
  digitalWrite(motorBL_1, LOW);
  digitalWrite(motorBL_2, HIGH);
  analogWrite(motorBL_speed, stall_speed);
//  
  digitalWrite(motorBR_1, LOW);
  digitalWrite(motorBR_2, HIGH);
  analogWrite(motorBR_speed, stall_speed);

  Serial.println(analogRead(A0));
}
