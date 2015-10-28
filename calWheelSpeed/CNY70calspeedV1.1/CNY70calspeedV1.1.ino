int pinReceptor = A1;
int sensorVal;
int black = 0;
double counter = 0;
int timer = 0;
double rotation = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
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
