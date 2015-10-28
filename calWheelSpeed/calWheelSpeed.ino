int pinReceptor = A1;
int sensorVal;
int BorW=0;
int counter = 0;
int caltime = 0;
double rotation = 0;
double Wheelspeed = 0;


void increase(){
  counter++;
  }

double getWheelspeed(double rota){
    double Wspeed = 0.1*rota;
    return Wspeed;
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorVal = analogRead(pinReceptor);
  if(sensorVal>20)
    {
      BorW = 1;
    }
  else
    {
      BorW = 0;
      //if(the engine is working){increase();}
    }
    
    
  //Serial.println(sensorVal);
  Serial.println(BorW);
  Serial.println(counter);
  if(caltime = 10)
  {
      rotation = counter/12;
      Wheelspeed = getWheelspeed(rotation);
      caltime = 0;
      counter = 0;
    }
  
  delay(100);
  
}
