#include <PID_v1.h>

#include <Servo.h>
#include <PID_v1.h>
#define sonarPin1 5
#define sonarPin2 6 
long pulse1,pulse2, inches1,inches2;
int preverr;
int ierr = 0;
int encodercount = 0;

float Kd = 1; float Ki = 0.5;
float Kp = 2;  
  double set,input,output;
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
  /*for (int i =0; i < 360; i++){
    double rad = degToRad(i);
    double speedOffset = sin(rad) * maxSpeedOffset;
    //double wheelOffset = sin(rad) * maxWheelOffset;
    esc.write(90 + speedOffset);
    //wheels.write(90 + wheelOffset);
    delay(50);*/
    
    esc.write(70);
    wheels.write(90);
    delay(1000);

    if(detect(0) || detect(1)){
      
     esc.write(80);
    long sensorvalue1 = pulseIn(sonarPin1,HIGH);
    long inchvalue1 = sensorvalue1/147;
     long sensorvalue2 = pulseIn(sonarPin2,HIGH);
    long inchvalue2 = sensorvalue2/147;
      long inchesmem2,inchesmem1;
       // case n=0: {
        if(detect(0)){
          esc.write(90);
          int temp = wheels.read();
           wheels.write(180-temp);
//          double timedelay = wheels.readMicroseconds();
//         long errordis = inchesmem2-inchvalue2;
//          delay(errordis*timedelay/sin(degToRad(90-wheels.read())));
          wheels.write(90-wheels.read());
        }
        else if (detect(1)){
         esc.write(0);
          
          wheels.write(90+wheels.read());
          double timedelay = wheels.readMicroseconds();
         long errordis = inchesmem1-inchvalue1;
//          delay(errordis*timedelay/sin(degToRad(90-wheels.read())));
          wheels.write(180-(wheels.read()-90));
        }
        }
       // }
       // case n=1: {
         // wheels.write(180-wheels.read());
       //   timedelay = wheels.readMicroseconds();
        //  errordis = inchesmem2-inchvalue2;
         // delay(errordis*timedelay/sin(degToRad(90-wheels.read())));
       // }
      
    // set = 90;
    // input = wheels.read();
     //myPID.Compute();
     
    // if(wheels.read()!=90){
     //wheels.write(90);
     //wheels.write(90-output);
     Serial.println("Wheels.read");
     Serial.println(wheels.read());
     Serial.println(pulseIn(sonarPin2,HIGH));
     
     delay(1000);
    }
   /* if(detect(0)){
      int val = wheels.read();
      wheels.write(val+(90-val));
    }
    else if (detect(1)){
      int val = wheels.read();
      wheels.write(val-(90-val));
    }*/
    

boolean detect(int sensor){
  boolean result = false;long value1,value2;
  if (sensor == 0){pulse1 = pulseIn(sonarPin1, HIGH);
  //147uS per inch
  inches1 = pulse1/147;
   value1 = inches1 * 2.54;}
  if (sensor == 1){pulse2 = pulseIn(sonarPin2, HIGH);
  inches2 = pulse2/147;
  //change inches to centimetres
   value2 = inches2 * 2.54;}
  
  delay(500);
  if (value1 <= 20 || value2 <=20) {result = true;}
  return result;
  
}
 
void loop()
{
   oscillate();
}
