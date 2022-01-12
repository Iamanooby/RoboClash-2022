
//global variables shared across different files must be place here (principal file)

//from controller tab
//from auton tab
//from gyro tab


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  auton_setup();
  controller_setup();
//  expand();//waits for button press to continue

}

void loop() 
{
  controller_loop();
//auton();
}
