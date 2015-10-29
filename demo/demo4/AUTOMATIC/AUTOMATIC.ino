#include <SoftwareSerial.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include <I2C.h>
SoftwareSerial XBee(2, 3); // RX, TX

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

double input,output,setpoint;
PID pid0(&input,&output,&setpoint,3.0,0.05,0.0000839,DIRECT);
Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
int pos = 0;         // Position of the servo (degress, [0, 180])
int distanceright, distanceleft, distance, LidarLeft, LidarRight;    // Distance measured
int startupDelay = 1000; // time to pause at each calibration step
int triggerPin1 = 13;
long distance1;
int sensorPins[] = {2,3}; // Array of pins connected to the sensor Power Enable lines
int sensorPinsArraySize = 2; // The length of the array
const int anPin1 = 0;
unsigned long pulse_width;
int pinReceptor = A1;
int sensorVal;
int black = 0;
double counter = 0;
int timer = 0;
double rotation = 0;



void start_sensor(){
  digitalWrite(triggerPin1,HIGH);
  delay(1);
  digitalWrite(triggerPin1,LOW);
}


void calSpeed(){
  black = 0;
  timer++;
  sensorVal = analogRead(pinReceptor);
  Serial.println(sensorVal);
  if(sensorVal > 100)  //the analog output is less than 100 when 
  {                    //sensor detects black object
    black = 1;
  }
  
  if(black == 1)
  {
    counter++;
    //Serial.print("counter:");
    //Serial.println(counter);
  }
  //Serial.println(black);
  if(timer == 20)    //print the speed of wheel every sec(20*50=1000ms)
  {
    rotation = counter/2.00;
    Serial.print("the wheel speed: ");
    Serial.print(rotation*18*3.14);
    Serial.println("cm/sec");
    timer = 0;
    counter = 0;
  }
  delay(50);
}

void setup()
{
  // Serial out
 
  Serial.begin(9600);

  // Servo control
//  myservo.attach(5); 
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  calibrateESC();
  
  // LIDAR control
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  for (int i = 0; i < sensorPinsArraySize; i++){
    pinMode(sensorPins[i], OUTPUT); // Pin to first LIDAR-Lite Power Enable line
    Serial.print(sensorPins[i]);
  }
  //PID Control
  pid0.SetOutputLimits(0,45 );
  pid0.SetMode(AUTOMATIC);
  setpoint=45;
  pinMode(triggerPin1, OUTPUT);
}


void calibrateESC(){
    esc.write(180); // full backwards
    delay(startupDelay);
    esc.write(0); // full forwards
    delay(startupDelay);
    esc.write(90); // neutral
    delay(startupDelay);
    esc.write(90); // reset the ESC to neutral (non-moving) value
}


void loop()
{  
  //distance for sonar sensor
  start_sensor();
  distance1 = analogRead(anPin1)/2;
  delay(50);
  if(distance1 < 10){esc.write(90);}
  if(distance1 > 10){
  enableDisableSensor(3); //Turn on sensor attached to pin 3 and disable all others
  double dis3 = readDistance()/2.54;
  enableDisableSensor(2);
  double dis2 = readDistance()/2.54;
  input = radToDeg(atan(abs(dis2 - dis3)/6.5));
    setpoint = 45;
    Serial.print(input);
    Serial.print("       ");
    pid0.Compute();
    Serial.println(output);
    //if (dis2 < 70  && dis3 < 70){
    if ((dis2 > dis3) && (wheels.read()+ 10 <90))
    { wheels.write(wheels.read()+10);}
    else if(dis2>dis3)
     { wheels.write(abs(wheels.read()-output));
    delay(10);   
   
    }
    if ((dis3 > dis2) && (wheels.read()-10 >90))
    {
      wheels.write(wheels.read()-10);}
      else if (dis3>dis2){
      wheels.write(abs(wheels.read()+output));
   
      }
    
    //else wheels.write(output);
    esc.write(75);
    calSpeed();
  delay(10);
  }

}
 /* Convert degree value to radians */
double degToRad(double degrees){
  return (degrees * 71) / 4068;
}

/* Convert radian value to degrees */
double radToDeg(double radians){
  return (radians * 4068) / 71;
}

void enableDisableSensor(int sensorPin){
  for (int i = 0; i < sensorPinsArraySize; i++){
      digitalWrite(sensorPins[i], LOW); // Turn off all sensors
  }
  digitalWrite(sensorPin, HIGH); // Turn on the selected sensor
  delay(1); // The sensor takes 1msec to wake
}

int readDistance(){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  
  return distance;   // Print Sensor Name & Distance
    
}
