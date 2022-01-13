#include <IBusBM.h>
#include <Servo.h>


////////////////////////////////////////////MOTORS/////////////////////////////////////

// Positive Terminal to odd number of L298N
// Negative Terminal to even number of L298N

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

class Motor 
{
  private:
    int in1_port, in2_port,pwm_port;
    int maxspeed = 222;
  
  public:
    Motor(int in1_port, int in2_port, int pwm_port)
    {
      this->in1_port= in1_port;
      this->in2_port= in2_port;
      this->pwm_port= pwm_port;
    }

    void init()
    {
      pinMode(this->in1_port, OUTPUT);
      pinMode(this->in2_port, OUTPUT);
      pinMode(this->pwm_port, OUTPUT); 
    }

    void moveMotor(int power_raw)
    {
      int power = constrain(power_raw,-100,100);//must have constraint or when it goes over the upper bound, it will go inverted
      int spd = map(abs(power), 0, 100, 0, maxspeed);//CHANGE BACK MAX SPEED
      analogWrite(this->pwm_port, spd);
      if (power==0)
      {
          digitalWrite(this->in1_port, LOW);
          digitalWrite(this->in2_port, LOW);

      }
      else if (power >0)
      {
          digitalWrite(this->in1_port, HIGH);
          digitalWrite(this->in2_port, LOW);      
      }
      else if (power <0)
      {
          digitalWrite(this->in1_port, LOW);
          digitalWrite(this->in2_port, HIGH);      
      }
    }
};//remember this semi colon

//motor declerations found in 1st principle tab/file
Motor motorFL(motorFL_1, motorFL_2, motorFL_speed);
Motor motorBL(motorBL_1,motorBL_2,motorBL_speed);

Motor motorFR(motorFR_1, motorFR_2, motorFR_speed);
Motor motorBR(motorBR_1,motorBR_2,motorBR_speed);

Motor motorIR(motorIR_1, motorIR_2, motorIR_speed);
Motor motorIL(motorIL_1,motorIL_2,motorIL_speed);


void setup_motors()
{
  motorFL.init();
  motorBL.init();
  motorFR.init();
  motorBR.init();
  motorIR.init();
  motorIL.init();
}

void move_robot(int FL_pow, int FR_pow, int BL_pow, int BR_pow)//this is for auton to control motors
{
    motorFL.moveMotor(FL_pow);
    motorFR.moveMotor(FR_pow);
    motorBL.moveMotor(BL_pow);
    motorBR.moveMotor(BR_pow);
}

void intake(int intake_pow)
{
    motorIL.moveMotor(intake_pow);
    motorIR.moveMotor(intake_pow);
}

////////////////////////////////////////////SERVOS/////////////////////////////////////

int servo_RightLift_pin = 3;
int servo_LeftLift_pin = 2;
int servo_Claw_pin = 4;
int servo_ClawArm_pin = 5;


Servo servo_RightLift,servo_LeftLift,servo_Claw,servo_ClawArm;

void setup_servo()
{
  servo_LeftLift.attach(servo_LeftLift_pin);
  servo_RightLift.attach(servo_RightLift_pin);
  lift_down();
  
  servo_Claw.attach(servo_Claw_pin);
  released();
  
  servo_ClawArm.attach(servo_ClawArm_pin);
  arm_collapsed();
}

int top_lift_pos = 120;//need to change
int bottom_lift_pos = 20;//need to change


void lift(int ch_value)
{
  int pos = map(ch_value, -100, 100, bottom_lift_pos, top_lift_pos);
  servo_LeftLift.write(pos);
  servo_RightLift.write(135-pos);    
}

void lift_up()
{
  servo_LeftLift.write(top_lift_pos);
  servo_RightLift.write(135-top_lift_pos);    
  
}

void lift_down()
{
  servo_LeftLift.write(bottom_lift_pos);
  servo_RightLift.write(135-bottom_lift_pos);    
}


void grabbed()
{
  servo_Claw.write(90);
}

void released()
{
  servo_Claw.write(30);
}


int top_arm_pos = 30;//need to change
int middle_arm_pos = 50;
int bottom_arm_pos = 170;//need to change
int collapsed_arm_pos = 2;//need to change 

void arm_up()
{
  servo_ClawArm.write(top_arm_pos);
}

void arm_mid()
{
  servo_ClawArm.write(middle_arm_pos);
}

void arm_down()
{
  servo_ClawArm.write(bottom_arm_pos);
  
}

void arm_collapsed()
{
  servo_ClawArm.write(collapsed_arm_pos);
}

//if joshua prefers manual control
void arm_control(int ch_value)
{

  int pos;
  if(ch_value>0)
  {
    pos = map(ch_value, 0, 100, middle_arm_pos, top_arm_pos);
    servo_ClawArm.write(pos);
  }
  else if(ch_value<=0)
  {
    pos = map(ch_value, -100, 0, bottom_arm_pos, middle_arm_pos);
    servo_ClawArm.write(pos);
  }
  
}

////////////////////////////////////////////CONTROLS/////////////////////////////////////


HardwareSerial& ibusRcSerial = Serial2;//use pin 15 to receiver for ibus
IBusBM ibusRc;

void setup_ibus()
{
  ibusRc.begin(ibusRcSerial);  
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue){
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

/////////////////////////////////////Field Oriented Control////////////////////

bool FOC_on = true;

void field_oriented_control(int &F, int &S)//FOC
{
  unreset_gyro();//remove this once u establish a button to reset gyro. otherwise this will use default forward position as 0. so face robot in normal orientation before turning robot on (drifting will occur)
  float theta_deg = gyro_loop();
  float theta_rad = theta_deg/180.0*M_PI;//convert back to rads
  float f_prime = F*cos(theta_rad)+S*sin(theta_rad);
  float s_prime = -F*sin(theta_rad)+S*cos(theta_rad);

  Serial.print("Theta:\t");
  Serial.print(theta_deg);
  Serial.print("F_prime:\t");
  Serial.print(f_prime);
  Serial.print("S_prime:\t");
  Serial.println(s_prime);

  //pass back by reference
  F = f_prime;
  S = s_prime;
}


///////////////////////////////////////////controller code/////////////////////////////////



void controller_setup()
{
  setup_motors();
  setup_servo();
  setup_ibus();


}

//Channel 2 – Left Stick, Up/Down 
//Channel 4 – Left Stick, Left/Right 
//
//Channel 1 – Right Stick, Left/Right 
//Channel 3 – Right Stick, Up/Down 
// 
//Channel 6 – Control VRA (left roller) inwards is +ve
//Channel 7 – Control VRB (right roller)inwards is +ve
//Channel 8 - Switch A
//Channel 9 - Left Button
//Channel 10- Right Button

int threshold = 5;
int ch_values [11] = { };//start using from index 1. So ch 1 is ch_values[1]. Range is from - 100 to 100
int stall_speed = 40;
int stall_time = 10; //in seconds

void controller_loop()
{
  for (byte i = 0; i<10; i++)
  {
    int ch_value = readChannel(i, -100, 100, 0);

    if (abs(ch_value)>threshold || (i!= 0 && i!= 1 && i!= 3 && i!=5 && i!=8))//set basd on whichever channels need thresholding
      ch_values [i+1] = ch_value;
    else
      ch_values [i+1] = 0;
  }



  if(FOC_on && ch_values[10]>0)//alter channel values temporarily, passed by reference
  {
     field_oriented_control(ch_values[2], ch_values[4]);//FOC
  }
  

  if (currSafe_check())//current is safe or safety is disabled
  {
    
    if(ch_values[1]==0 && ch_values[2]==0 && ch_values[4]==0 && ch_values[6]==0 && ( millis()%(stall_time*1000)<=10 ))
    {
      //stallcode for 10ms when no motors running to keep baseus working for every intervalstall time
      move_robot(stall_speed, stall_speed, -stall_speed, stall_speed);//strafe left
    }
    else
    {
      //run base motor motor
  
      motorFL.moveMotor(+ch_values[2]+ch_values[4]+ch_values[1]);
      motorFR.moveMotor(+ch_values[2]-ch_values[4]-ch_values[1]);
      motorBL.moveMotor(+ch_values[2]-ch_values[4]+ch_values[1]);
      motorBR.moveMotor(+ch_values[2]+ch_values[4]-ch_values[1]);
    }
  }
  else
  {
    move_robot(0,0,0,0);//stop
//      delay(500);//let current drop before proceeding
  }
  //run intake motor
  motorIR.moveMotor(ch_values[6]);
  motorIL.moveMotor(ch_values[6]);


  //control lift
  lift(ch_values[3]);

//    //control arm
//    if (ch_values[7]>0)
//      arm_up();
//    else if (ch_values[7]<0)
//      arm_down();
//    else;
//      //remain status quo


  //control claw
  if (ch_values[9]<0)
    grabbed();
  else if (ch_values[9]>0)
    released();

  //for auton, check for button pressed AND trigger then activate

  //channel 8 already thresholded
  if(ch_values[8]<0)//default, expands
  {
    if (button_pressed())
   {
    expand();
   }
  }
  else if(ch_values[8]==0)//run auton
  {
    if(button_pressed())
    {
      auton();
    }
  }
  else
  {
    //press button nothing happens, instead run normal arm mapping
    arm_control(ch_values[7]);
  }





}
