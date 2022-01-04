#include <IBusBM.h>
#include <Servo.h>


////////////////////////////////////////////MOTORS/////////////////////////////////////

// Positive Terminal to odd number of L298N
// Negative Terminal to even number of L298N

int motorFL_speed = 6;            //front left enA
int motorFL_1 = 23;                //front left in1
int motorFL_2 = 25;                //front left in2
int motorBL_speed = 7;            //back left enB
int motorBL_1 = 27;                //back left in3
int motorBL_2 = 29;                //back left in4


int motorFR_speed = 8;            //front right enA
int motorFR_1 = 31;                //front right in1
int motorFR_2 = 33;                //front right in2
int motorBR_speed = 9;           //back right enB
int motorBR_1 = 35;               //back right in3
int motorBR_2 = 37;               //back right in4

int motorIL_speed = 10;            //intake left enA, unused
int motorIL_1 = 39;                //intake left in1
int motorIL_2 = 41;                //intake left in2
int motorIR_speed = 11;           //intake right enB, unused
int motorIR_1 = 43;               //intake right in3
int motorIR_2 = 45;               //intake right in4

class Motor 
{
  private:
    int in1_port, in2_port,pwm_port;
  
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

    void moveMotor(int power)
    {
      int spd = map(abs(power), 0, 100, 0, 255);
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
    motorFL.moveMotor(intake_pow);
    motorFR.moveMotor(intake_pow);
}

////////////////////////////////////////////SERVOS/////////////////////////////////////

int servo_RightLift_pin = 2;
int servo_LeftLift_pin = 3;
int servo_Claw_pin = 4;
int servo_ClawArm_pin = 5;


Servo servo_RightLift,servo_LeftLift,servo_Claw,servo_ClawArm;

void setup_servo()
{
  servo_LeftLift.attach(servo_LeftLift_pin);
  servo_RightLift.attach(servo_RightLift_pin);
  servo_Claw.attach(servo_Claw_pin);
  servo_ClawArm.attach(servo_ClawArm_pin);
}

void lift(int ch_value)
{
  int bottom_lift_pos = 0;
  int top_lift_pos = 180;
  int pos = map(ch_value, -100, 100, bottom_lift_pos, top_lift_pos);
  servo_LeftLift.write(pos);
  servo_RightLift.write(145-pos);    
}

void grabbed()
{
  servo_Claw.write(90);
}

void released()
{
  servo_Claw.write(0);
}

void arm_up()
{
  servo_ClawArm.write(90);
}

void arm_down()
{
  servo_ClawArm.write(0);
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




///////////////////////////////////////////controller code/////////////////////////////////



void controller_setup()
{
  setup_motors();
  setup_servo();
  setup_ibus();


}

void field_oriented_control()
{
  //do next time if needed
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


void controller_loop()
{
    for (byte i = 0; i<10; i++)
    {
      int ch_value = readChannel(i, -100, 100, 0);

      if (abs(ch_value)>threshold || (i!= 0 && i!= 1 && i!= 3 && i!=5 && i !=6))//set basd on whichever channels need thresholding
        ch_values [i+1] = ch_value;
      else
        ch_values [i+1] = 0;
    }

    //run base motor motor
    motorFL.moveMotor(+ch_values[2]+ch_values[4]+ch_values[1]);
    motorFR.moveMotor(+ch_values[2]-ch_values[4]-ch_values[1]);
    motorBL.moveMotor(+ch_values[2]-ch_values[4]+ch_values[1]);
    motorBR.moveMotor(+ch_values[2]+ch_values[4]-ch_values[1]);
    //run intake motor
    motorIR.moveMotor(ch_values[6]);
    motorIL.moveMotor(-ch_values[6]);

    //control lift
    lift(ch_values[3]);

    //control arm
    if (ch_values[7]>0)//remember, already thresholded
      arm_up();
    else if (ch_values[7]<0)
      arm_down();
    else;
      //remain status quo

    //control claw
    if (ch_values[9]>0)
      grabbed();
    else if (ch_values[10]>0)
      released();

    //for auton, check for button pressed AND trigger then activate
    if (ch_values[8]>0)
    {
      if(button_pressed())
      {
        auton();
      }
    }

}