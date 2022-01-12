#include <Servo.h> 

Servo servo_RightLift,servo_LeftLift,servo_Claw,servo_ClawArm;

int servo_RightLift_pin = 3;
int servo_LeftLift_pin = 2;
int servo_Claw_pin = 4;
int servo_ClawArm_pin = 5;


void setup() 
{ 
//  servo_Claw.attach(servo_Claw_pin);
//// 30 is claw open
////80 is closed on cube
////  95 is fully closed
//servo_Claw.write(30);

//  servo_ClawArm.attach(servo_ClawArm_pin);
////170 is claw arm down
////50 is claw arm vertical rest
////30 is claw arm release point
//servo_ClawArm.write(50);



  servo_LeftLift.attach(servo_LeftLift_pin);
  servo_RightLift.attach(servo_RightLift_pin);
  int pos = 120;
  //up is 120
  //down is 30
  servo_LeftLift.write(pos);
  servo_RightLift.write(145-pos);  
} 

void loop() {} 


//
//  claw.write(60);
////  delay(1000);
//  claw_arm.write(180);
//  delay(10000000);
//  claw.write(80);
//  delay(1000);
//  claw_arm.write(30);  
//  delay(3000);
//  claw.write(30);
