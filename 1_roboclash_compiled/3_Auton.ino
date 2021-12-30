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
const int trigPin = 49;
const int echoPin = 51;

void setup_ultrasound()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

long ult_distance () {
  long dur;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); // delays are required for a succesful sensor operation.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); //this delay is required as well!
  digitalWrite(trigPin, LOW);
  dur = pulseIn(echoPin, HIGH);
  return (dur * 0.034 / 2);// convert the distance to centimeters.
  }



///////////////////////////////PID///////////////////////////////////////////////
ArduPID myController;




double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 512;
double p = 10;
double i = 1;
double d = 0.5;

void pid_setup()
{
  myController.begin(&input, &output, &setpoint, p, i, d);

  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  myController.setOutputLimits(-100, 100);
  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  
  myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
}

float PID_gyro_loop()
{
  input = 0; // Replace with sensor feedback aka gyro
  myController.compute();
  return output;
}

////////////////////////////////////movement///////////////////////////////////
void moving(char dir, int power)//remember to reset setpoint as current point before the loop this is inside. aka target = current
{

  int adjust = PID_gyro_loop();
  if (dir == 'F')
  {
    move_robot(power-adjust, power+adjust, power-adjust, power+adjust);
  }
  else if (dir == 'B')
  {
    move_robot(-power-adjust, -power+adjust, -power-adjust, -power+adjust);
  }
  else if (dir == 'R')
  {
    move_robot(power-adjust, -power+adjust, -power+adjust, power+adjust);
  }
  else if (dir == 'L')
  {
    move_robot(-power-adjust, power+adjust, power-adjust, -power+adjust);
  }
}

////////////////////////////////////auton////////////////////////////////////

void setup_auton()
{
  setup_button();
  setup_ultrasound();
  pid_setup();
}

void auton()
{
  //if button pressed then execute
}
