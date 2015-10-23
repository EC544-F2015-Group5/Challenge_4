#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial XBee(2, 3); // RX, TX


Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'
double wheelOffset = 0.0; // For Adjusting the wheel

//double wheelStartUpOffset = 0.0; // For Adjusting the steering
char buffer;
//define constants
#define LED 4

void setup() {
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9                
  XBee.begin(9600);
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  
//inilitialization
  digitalWrite(LED, LOW);
}
/* Convert degree value to radians */
double degToRad(double degrees){
  return (degrees * 71) / 4068;
}

/* Convert radian value to degrees */
double radToDeg(double radians){
  return (radians * 4068) / 71;
}
/* Calibrate the ESC by sending a high signal, then a low, then middle.*/
void calibrateESC(){
    esc.write(180); // full backwards
    delay(startupDelay);
    esc.write(0); // full forwards
    delay(startupDelay);
    esc.write(90); // neutral
    delay(startupDelay);
    esc.write(90); // reset the ESC to neutral (non-moving) value
}

/* Oscillate between various servo/ESC states, using a sine wave to gradually 
 *  change speed and turn values.
 */
void oscillate(){
  for (int i =0; i < 360; i++){
    double rad = degToRad(i);
    double speedOffset = sin(rad) * maxSpeedOffset;
    double wheelOffset = sin(rad) * maxWheelOffset;
    esc.write(90 + speedOffset);
    wheels.write(90 + wheelOffset);
    delay(50);
  }
}


void loop() {
  while(XBee.available()){
    buffer = XBee.read();
    //delay time for serialport to read data
    delay(3);
    if (buffer == '1'){
      digitalWrite(LED, HIGH);
      XBee.println("Foward");
      esc.write(0); // full fowards
      delay(1000);
      continue;
    }
    else if (buffer == '2'){
      digitalWrite(LED, HIGH);
      XBee.println("Stop");
      esc.write(90); // full backwards
      delay(1000);
      continue;
    }
    else if (buffer == '3'){
      digitalWrite(LED, HIGH);
      XBee.println("Turn Right");
      esc.write(0); 
      wheels.write(45); // left
      continue;
    }
    else if (buffer == '4'){
      digitalWrite(LED, HIGH);
      XBee.println("Turn Left");
      wheels.write(135); // right
      continue;
    }
}
}