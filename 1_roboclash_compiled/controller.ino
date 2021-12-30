#include <IBusBM.h>
#include <Servo.h>


////////////////////////////////////////////MOTORS/////////////////////////////////////

// Positive Terminal to odd number of L298N
// Negative Terminal to even number of L298N

int motorBL_speed = 11;            //front left enA
int motorBL_1 = 13;                //front left in1
int motorBL_2 = 12;                //front left in2
int motorFL_speed = 5;            //back left enB
int motorFL_1 = 4;                //back left in3
int motorFL_2 = 7;                //back left in4

int motorFR_speed = 6;            //front right enA
int motorFR_1 = 8;                //front right in1
int motorFR_2 = 9;                //front right in2
int motorBR_speed = 10;           //back right enB
int motorBR_1 = 11;               //back right in3
int motorBR_2 = 12;               //back right in4


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

Motor motorFL(motorFL_1, motorFL_2, motorFL_speed);
Motor motorBL(motorBL_1,motorBL_2,motorBL_speed);

Motor motorFR(motorFR_1, motorFR_2, motorFR_speed);
Motor motorBR(motorBR_1,motorBR_2,motorBR_speed);

////////////////////////////////////////////SERVOS/////////////////////////////////////
int servo_LeftLift_pin = 2;
int servo_RightLift_pin = 3;

Servo servo_LeftLift, servo_RightLift;


void lift(int ch_value)
{
  int bottom_lift_pos = 0;
  int top_lift_pos = 180;
  int pos = map(ch_value, -100, 100, bottom_lift_pos, top_lift_pos);
  servo_LeftLift.write(pos);
  servo_RightLift.write(145-pos);    
}

////////////////////////////////////////////CONTROLS/////////////////////////////////////


HardwareSerial& ibusRcSerial = Serial;
IBusBM ibusRc;

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


void field_oriented_control()
{
  //do next time if needed
}




void controller_setup()
{
  motorFL.init();
  motorBL.init();
  motorFR.init();
  motorBR.init();

  servo_LeftLift.attach(servo_LeftLift_pin);
  servo_RightLift.attach(servo_RightLift_pin);


  ibusRc.begin(ibusRcSerial);  
}

//Channel 2 – Left Stick, Up/Down 
//Channel 4 – Left Stick, Left/Right 
//
//Channel 1 – Right Stick, Left/Right 
//Channel 3 – Right Stick, Up/Down 
// 
//Channel 6 – Control VRA inwards is +ve
//Channel 7 – Control VRB
//Channel 8 - Switch A

int threshold = 5;
int ch_values [9] = { };//start using from index 1. So ch 1 is ch_values[1]. Range is from - 100 to 100


void controller_loop()
{
    for (byte i = 0; i<7; i++)
    {
      int ch_value = readChannel(i, -100, 100, 0);

      if (abs(ch_value)>threshold || (i!= 0 && i!= 1 && i!= 3))//set basd on whichever channels need thresholding
        ch_values [i+1] = ch_value;
      else
        ch_values [i+1] = 0;
    }

    //run motor
    motorFL.moveMotor(+ch_values[2]+ch_values[4]+ch_values[1]);
    motorFR.moveMotor(+ch_values[2]-ch_values[4]-ch_values[1]);
    motorBL.moveMotor(+ch_values[2]-ch_values[4]+ch_values[1]);
    motorBR.moveMotor(+ch_values[2]+ch_values[4]-ch_values[1]);

    lift(ch_values[3]);
}
