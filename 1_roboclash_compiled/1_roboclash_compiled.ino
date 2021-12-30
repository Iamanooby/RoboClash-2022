
//global variables shared across different files must be place here (principal file)

//from controller tab


//from auton tab

//from gyro tab


void setup() {
  // put your setup code here, to run once:
  setup_auton();
  controller_setup();

}

void loop() 
{
  controller_loop();

}
