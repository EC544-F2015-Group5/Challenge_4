#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial xbee(2,3);


#define ESCPin 9
#define servoPin 5
#define minSignal 1000
#define maxSignal 2000



#define sonarPin1 6
#define sonarPin2 7
long pulse1,pulse2, inches1,inches2;
int angle;

//Maxbotix sonarPin1(A0, Maxbotix::AN, Maxbotix::LV);
//Maxbotix sonarPin2(A1, Maxbotix::AN, Maxbotix::LV);

//defines directions
const int DirLeft = 0;
const int DirRight = 1;
//defines obstacle constants
const int OBST_zero = 0;
const int OBST_Left = 1;
const int OBST_Right = 2;
const int OBST_Front = 3;

Servo ESC;
Servo servo;



void setup() {
  
  // put your setup code here, to run once:
  xbee.begin(9600);
  Serial.begin(9600);
  //initialize ESC 
  ESC.attach(ESCPin, minSignal, maxSignal);
  servo.attach(servoPin);
  ESC.writeMicroseconds(maxSignal);
  while(!Serial.available());
  Serial.read();
  ESC.writeMicroseconds(minSignal);
  
  
  pinMode(sonarPin1, INPUT);
  pinMode(sonarPin2, INPUT);
  angle = servo.read();
}


void loop() {
    unsigned long start;
    
  // put your main code here, to run repeatedly:
  // If there is a command retrieved from
  if (xbee.available() > 0) {   // check if there is input msg
    String msg  = "";

    // Read in message
    while(xbee.available() > 0) {
      msg += char(xbee.read());
    }
    if (msg.equals("Start\n")){
      ESC.writeMicroseconds(1700);
      
    }
    else if (msg.equals("Stop\n")) {
      ESC.writeMicroseconds(1500);  //stopped signal
    }
    }
     //initial angle of servo;
     
    if(obstacleScan(OBST_Left)==true){
      //To Do call move backwards and turn right then move straight
      int val = servo.read();
      servo.write(val+(angle-val));
      
    }

    if(obstacleScan(OBST_Right) == true){
      //To Do call move backwards and turn left then move straight

      int val = servo.read();
      servo.write(val-(angle-val));
    }
       
}

//void SensorBegin(){
   //start = millis();
   //sonarPin1.getRange();

//}
boolean obstacleScan(int obstacle){
  switch(obstacle){
  case OBST_Front: return detect(DirLeft) && detect(DirRight);
  case OBST_Left: return detect(DirLeft);
  case OBST_Right:return detect(DirRight);   
}
  return false;
}

boolean detect(int sensor){
  boolean result = false;long value1,value2;
  if (sensor == 0){pulse1 = pulseIn(sonarPin1, HIGH);
  //147uS per inch
  inches1 = pulse1/147;
   value1 = inches1 * 2.54;}
  else if (sensor == 1){pulse2 = pulseIn(sonarPin2, HIGH);
  inches2 = pulse2/147;
  //change inches to centimetres
   value2 = inches2 * 2.54;}
  
  delay(500);
  if (value1 <= 8 || value2 == 0)result = true;
  return result;
  
}

