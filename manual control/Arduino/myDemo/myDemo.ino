/*
*
* Autonomous control
*/


int irsensor = A5;
int motorspeed;
int run = 0;
int measure = 1;
int ambientir = 0;
int distance;

void setup() {
  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin
  pinMode(irsensor, INPUT);
  digitalWrite(irsensor, HIGH);
  Serial.begin(9600);
}

void loop(){
  if(run == 0)
  {
    delay(1000);
    do{
      ambientir = ambientir + analogRead(irsensor);
      delay(1000);
      measure = measure + 1;
    }
    while(measure < 10);
    ambientir = ambientir / 10;
    run = run +1;
  }

  distance = analogRead(irsensor);

  if(distance < ambientir - 50){
    digitalWrite(12, HIGH); //Establishes backward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 100);   //Spins the motor on Channel A at half speed
  }
 
  if(distance > ambientir - 50){
    digitalWrite(12, LOW); //Establishes forward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 100);   //Spins the motor on Channel A at full speed
  }
  Serial.println(distance);
}