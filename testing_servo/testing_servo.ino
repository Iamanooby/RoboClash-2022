#include <Servo.h> 

Servo claw,claw_arm;

void setup() 
{ 
  claw.attach(2);
//  myservo.write(30);  // set servo to mid-point
// 30 is claw open
//80 is closed on cube
//  95 is fully closed


  claw_arm.attach(3);
//  myservo2.write(30);  // set servo to mid-point
//170 is claw arm down
//50 is claw arm vertical rest
//30 is claw arm release point


  claw.write(30);
  delay(1000);
  claw_arm.write(170);
  delay(5000);
  claw.write(80);
  delay(1000);
  claw_arm.write(30);  
  delay(3000);
  claw.write(30);
} 

void loop() {} 
