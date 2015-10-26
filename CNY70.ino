#include <TimerOne.h>
unsigned int counter=0;
 
int pinReceptor=A1;  // 
int sensorVal;

void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
} 
 
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  Serial.print("Motor Speed: "); 
  int rotation = (counter/12);  // divide by number of holes in Disc
  Serial.print(rotation,DEC);  
  Serial.println(" Rotation per seconds"); 
  counter=0;  //  reset counter to zero
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}
 
void setup() 
{
  Serial.begin(9600);
  
 pinMode(A1, INPUT); 
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(0, docount, CHANGE);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
} 
 
void loop()
{
  sensorVal = analogRead(pinReceptor);
  if (sensorVal>20)
  {
    docount();
  }
//  analogWrite(A1, motorspeed);  // set speed of motor (0-255)
  //digitalWrite(A1, 1);  // set rotation of motor to Clockwise
}
