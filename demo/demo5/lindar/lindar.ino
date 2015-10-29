#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include <I2C.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

//PID Mode
double input,output,setpoint;
PID pid0(&input,&output,&setpoint,3,0,0.9,DIRECT);

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
int count;

/* Start the ultra sonic sensor*/
void start_sensor(){
  digitalWrite(triggerPin1,HIGH);
  delay(1);
  digitalWrite(triggerPin1,LOW);
}

/*Caculate the speed of car*/
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
    Serial.print(rotation);
    Serial.println("rot/sec");
    Serial.print(rotation*18*3.14);
    Serial.println("cm/sec");
    timer = 0;
    counter = 0;
  }
  delay(50);
}

void setup()
{
  
  Serial.begin(9600);

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
  pid0.SetOutputLimits(-50, 50);
  pid0.SetMode(AUTOMATIC);
  setpoint=50;
  pinMode(triggerPin1, OUTPUT);
  esc.write(75);
}

/* Calibration */
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
  enableDisableSensor(2); //Turn on sensor attached to pin 2 and disable all others
  input=readDistance();
  if (input > 100) {
    setpoint = input;
  }
  else {
    setpoint = 50;
  }
    Serial.print(input);
    Serial.print("       ");
    pid0.Compute();
    Serial.println(output);
    wheels.write(90 + output);
    delay(10);
    calSpeed();
    start_sensor();
    distance1 = analogRead(anPin1)/2;
    delay(50);
    if(distance1<20){
      count++;
    }else{
      count=0;
    }
    if(count==3){
      count=0;
      esc.write(90);
    }
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
