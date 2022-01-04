
//global variables shared across different files must be place here (principal file)

//from controller tab
//from auton tab
//from gyro tab


void setup() {
  // put your setup code here, to run once:
//  auton_setup();
//  controller_setup();
pid_setup();
Serial.begin(115200);
}

void loop() 
{
//  controller_loop();
Serial.println(PID_gyro_loop());
}
