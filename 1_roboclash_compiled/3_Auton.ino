#include <ArduPID.h>


////////////////////////////////////////////////button///////////////////////////////////////
int button_pin = 47;

void setup_button()
{
  pinMode(button_pin,INPUT_PULLUP);//default is HIGH
}

bool button_pressed()
{
  return !digitalRead(button_pin);
}
// i dont think need debouncing, cos its used to count number of times button is pressed.
//for checking in auton,
//just detect button press to stop
//delay 20 ms
//check for release
//delay 20ms
//wait for button press to continue

//lift, grab and arm use functions in controller tab


//////////////////////////////////////////ultrasound//////////////////////////////////////////
const int LB_trigPin = A6;
const int LB_echoPin = A5;
const int RF_trigPin = A4;
const int RF_echoPin = A3;

void setup_ultrasound()
{
  pinMode(LB_trigPin, OUTPUT);
  pinMode(LB_echoPin, INPUT);

  pinMode(RF_trigPin, OUTPUT);
  pinMode(RF_echoPin, INPUT);
}

long LB_ult_distance () {
  long dur;
  digitalWrite(LB_trigPin, LOW);
  delayMicroseconds(2); // delays are required for a succesful sensor operation.
  digitalWrite(LB_trigPin, HIGH);
  delayMicroseconds(10); //this delay is required as well!
  digitalWrite(LB_trigPin, LOW);
  dur = pulseIn(LB_echoPin, HIGH);
  return (dur * 0.034 / 2);// convert the distance to centimeters.
  }

long RF_ult_distance () {
  long dur;
  digitalWrite(RF_trigPin, LOW);
  delayMicroseconds(2); // delays are required for a succesful sensor operation.
  digitalWrite(RF_trigPin, HIGH);
  delayMicroseconds(10); //this delay is required as well!
  digitalWrite(RF_trigPin, LOW);
  dur = pulseIn(RF_echoPin, HIGH);
  return (dur * 0.034 / 2);// convert the distance to centimeters.
  }

//////////////////////////////IR/////////////////////////////////////////////////

const int RIR = A2;
const int LIR = A1;

int RIR_value()
{
  return analogRead(RIR);
}

int LIR_value()
{
  return analogRead(LIR);
}

int RIR_black_val = 600;//blacker means higher value
int LIR_black_val = 600;


bool RIR_line()
{
  return RIR_value()> RIR_black_val;
}

bool LIR_line()
{
  return LIR_value()> LIR_black_val;
}

///////////////////////////////GYRO PID///////////////////////////////////////////////
ArduPID gyro_pid_Controller;

double gyro_pid_input;
double gyro_pid_output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double gyro_pid_setpoint = 0;
double gyro_pid_p = 0.1;
double gyro_pid_i = 0;
double gyro_pid_d = 0.05;

void gyro_pid_setup()
{
  gyro_setup();
  gyro_pid_Controller.begin(&gyro_pid_input, &gyro_pid_output, &gyro_pid_setpoint, gyro_pid_p, gyro_pid_i, gyro_pid_d);//p, i, d

  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  gyro_pid_Controller.setOutputLimits(-200, 200);
  gyro_pid_Controller.setBias(0);
  gyro_pid_Controller.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  
  gyro_pid_Controller.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
}

float gyro_pid_loop()
{
  gyro_pid_input = gyro_target_error(); 
  gyro_pid_Controller.compute();
  return -gyro_pid_output;
}

////////////////////////////////////gyro movement///////////////////////////////////
void moving(String dir, int power)//remember to soft_reset_gyro(0) before using this

{
  
  int adjust = gyro_pid_loop();
  if (dir == "F")
  {
    move_robot(power-adjust, power+adjust, power-adjust, power+adjust);//int FL_pow, int FR_pow, int BL_pow, int BR_pow
  }
  else if (dir == "B")
  {
    move_robot(-power-adjust, -power+adjust, -power-adjust, -power+adjust);
  }
  else if (dir == "R")
  {
    move_robot(power-adjust, -power+adjust, -power+adjust, power+adjust);
  }
  else if (dir == "L")
  {
    move_robot(-power-adjust, power+adjust, power-adjust, -power+adjust);
  }
  else if (dir == "RR")
  {
    move_robot(power, -power, power, -power);
  }
  else if (dir == "RL")
  {
    move_robot(-power, power, -power, power);
  }
  else if (dir == "S")
  {
    move_robot(0, 0, 0, 0);
  }
}

void turn_Gyro(int angle, int power)
{

  int threshold = 1;
  soft_reset_gyro(angle);//add angle to target based on robot current angle
  

  while (true)//threshold, can do a pid here if you want. currently is const
  {
    float turn = gyro_target_error();
    
    if (turn > threshold)//RR
    {
      move_robot(power, -power, power, -power);
    }
    else if (turn < -threshold)//RL
    {
      move_robot(-power, power, -power, power);
    }
    else
    {
      break;
    }
  }
  unreset_gyro();
  moving("S", 0);
}
///////////////////////////////Light PID///////////////////////////////////////////////
ArduPID light_pid_Controller;

double light_pid_input;
double light_pid_output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double light_pid_setpoint = 300;
double light_pid_p = 0.1;
double light_pid_i = 0;
double light_pid_d = 0.05;

void light_pid_setup()
{
  light_pid_Controller.begin(&light_pid_input, &light_pid_output, &light_pid_setpoint, light_pid_p, light_pid_i, light_pid_d);//p, i, d

  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  light_pid_Controller.setOutputLimits(-200, 200);
  light_pid_Controller.setBias(0);
  light_pid_Controller.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  
  light_pid_Controller.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
}

float light_pid_loop()
{
  light_pid_input = analogRead(A7); // Replace with sensor feedback aka gyro
  light_pid_Controller.compute();
  return light_pid_output;
}

//////////////////////////////////line tracking movement/////////////////////

void line_track(int power)//remember to soft_reset_gyro() before using this

{
  
  int adjust = light_pid_loop();

    move_robot(power-adjust, power+adjust, power-adjust, power+adjust);//int FL_pow, int FR_pow, int BL_pow, int BR_pow

}
////////////////////////////////////auton////////////////////////////////////

void auton_setup()
{
  setup_button();
  setup_ultrasound();
  gyro_pid_setup();
}


int slow_speed = 50;
int mid_speed = 70;
int fast_speed = 90;

void expand()//this will be done in setup
{
//  while(button_pressed()==false)//wait for button press
//  {
//    
//  }
//  Serial.println("Start Expand");

  //execute expand code
  arm_mid();

  move_robot(mid_speed, mid_speed, mid_speed, mid_speed);//dont want pid here
  delay(500);
  move_robot(-mid_speed, -mid_speed, -mid_speed, -mid_speed);//dont want pid here
  delay(1000);
  moving("S", 0);
}

void auton()
{
  //if button pressed then execute

  //move forward to intake
  intake(100);
  unsigned long timing = millis();
  while(millis()-timing<1000)
  {
    moving("F", slow_speed);//must be in loop for gyro
  }
  moving("S", 0);
  delay(1000);
  intake(0);
  
  //move back untill timing and button pressed
  move_robot(-mid_speed, -mid_speed, -mid_speed, -mid_speed);
  timing = millis();
  while(millis()-timing<3000 && button_pressed()==false);//3s cut off timing, if button still not pressed
  moving("S", 0);
  delay(1000); //wait 1s to release button
  



}
