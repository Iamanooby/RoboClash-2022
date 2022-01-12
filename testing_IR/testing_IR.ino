//const int digitalIn = 2;
const int analogIn = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  pinMode(digitalIn,INPUT);
  pinMode(analogIn,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Analog:\t");
  Serial.println(analogRead(analogIn));
//  Serial.print("\tDigital:\t");
//  Serial.println(digitalRead(digitalIn));
}
