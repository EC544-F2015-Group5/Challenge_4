#include <I2C.h>

//#include <i2cmaster.h>

#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#define sonarPin1 5
#define sonarPin2 6 
long pulse1,pulse2, inches1,inches2;
int preverr;
int ierr = 0;
int encodercount = 0;
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int reading = 0;
int sensorPins[] = {2,3};
int sensorPinsArraySize = 2;

float Kd = 3.0 ; float Ki = 0.05;
float Kp = 0.0000839;  double set; 
  double input,output;
  PID myPID(&input, &output, &set, Kp, Ki, Kd, DIRECT);
  const int samplerate = 1;
Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to use at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'
 
void setup()
{
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  /*  If you're re-uploading code via USB while leaving the ESC powered on, 
   *  you don't need to re-calibrate each time, and you can comment this part out.
   */
   
  calibrateESC();
  //set = wheels.read();
  //myPID.SetMode(AUTOMATIC);
  long pulsemem1 = pulseIn(sonarPin1,HIGH);
  long inchesmem1 = pulsemem1/147;
   long pulsemem2 = pulseIn(sonarPin2,HIGH);
  long inchesmem2 = pulsemem2/147;
  Serial.begin(9600);
  Wire.begin();
  double input = wheels.read();
  double set = 90;
  myPID.SetMode(AUTOMATIC);
  I2c.begin();
  delay(100);
  I2c.timeOut(50);
   for (int i = 0; i < sensorPinsArraySize; i++){
    pinMode(sensorPins[i], OUTPUT); // Pin to first LIDAR-Lite Power Enable line
    Serial.print(sensorPins[i]);
  }
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
   //esc.write(80);
  input = wheels.read();
        myPID.Compute();
       // wheels.write(wheels.read()-mem);
        wheels.write(output);
        enableDisableSensor(3);
        if (readDistance() < 82) {right();}
        enableDisableSensor(2);
        if (readDistance() <82) { left();   }
    //esc.write(80);
    //delay(1000);
    //pin 5 is right
   
   
       
      
   
     
    }
  void right(){
    int steer = wheels.read();
    if (steer - 10 > 90)
    wheels.write(steer-10);
    else wheels.write(90);
  }
    void left(){
    int steer = wheels.read();
    if (steer + 10 < 90)
    wheels.write(steer+10);
    else wheels.write(90);
    
  }

boolean detect(int sensor){
  boolean result = false;long value1,value2;
  if (sensor == 0){pulse1 = pulseIn(sonarPin1, HIGH);
  //147uS per inch
  inches1 = pulse1/147;
   }
  else if (sensor == 1){pulse2 = pulseIn(sonarPin2, HIGH);
  inches2 = pulse2/147;
  
   }
  
  delay(5);
  if (inches1 <= 32  || inches2 <= 32 ) {result = true;}
  return result;
  
}
 
void loop()
{
    esc.write(80);
   oscillate();
}
