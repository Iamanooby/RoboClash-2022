const int LB_trigPin = 3;
const int LB_echoPin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(LB_trigPin, OUTPUT);
  pinMode(LB_echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  long dur;
  digitalWrite(LB_trigPin, LOW);
  delayMicroseconds(2); // delays are required for a succesful sensor operation.
  digitalWrite(LB_trigPin, HIGH);
  delayMicroseconds(10); //this delay is required as well!
  digitalWrite(LB_trigPin, LOW);
  dur = pulseIn(LB_echoPin, HIGH);
Serial.println((dur * 0.034 / 2));// convert the distance to centimeters.
  
}
