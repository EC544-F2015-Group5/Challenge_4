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


float Kd = 0.75 ; float Ki = 15;
float Kp = 5.5;  double set; 
  double input,output;
  PID myPID(&input, &output, &set, Kp, Ki, Kd, DIRECT);
  const int samplerate = 1;
Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
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
   esc.write(75);
   // wheels.write(90);
    delay(1000);
    //pin 5 is right
      Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(2000); // Wait 20ms for transmit

  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.println(reading); // print the reading
    if(reading <=18)
    
  //}
    //if(detect(0)){
      int mem = wheels.read();
        
          wheels.write(90+wheels.read());
        
        // delay(2000);
          
        input = wheels.read();
        myPID.Compute();
       // wheels.write(wheels.read()-mem);
        wheels.write(output);
        }
        else if(detect(1)){
          int mem = wheels.read();
          wheels.write(180-mem);
         // delay(2000);
          input = wheels.read();
          myPID.Compute();
          wheels.write(output);
        }
       
      
     Serial.println("Wheels.read");
     Serial.println(wheels.read());
     Serial.println(pulseIn(sonarPin2,HIGH));
     
     delay(1000);
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
  if (value1 <= 18  || value2 <=18 ) {result = true;}
  return result;
  
}
 float getdegree(){
  Wire.beginTransmission(0x21);
  Wire.write("A");     // Send "Get Data" command, info from Datasheet
  delay(100);         // interface command delay, info from Datasheet
  Wire.requestFrom(0x21, 2); //get the two data bytes, MSB and LSB
  byte MSB = Wire.read(); // Result will be in tenths of degrees (0 to 3599)
  byte LSB = Wire.read(); // Provided in binary format over two bytes."
  Wire.endTransmission();
  // Compute result from the two bytes results
  float myres = ((MSB << 8) + LSB) / 10; 
  return myres;
  // Display result
  Serial.print(myres);
  Serial.println(" degrees");
  //delay(10);
}
void loop()
{
   oscillate();
}
