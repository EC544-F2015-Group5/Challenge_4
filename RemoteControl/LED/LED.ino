#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // RX, TX

char buffer;
//define constants
#define LED1 2
#define LED2 5
#define LED3 11
#define LED4 13

void setup() {                
  XBee.begin(9600);
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
//inilitialization
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
}

void loop() {
  while(XBee.available()){
    buffer = XBee.read();
    //delay time for serialport to read data
    delay(3);
    if (buffer == '1'){
      digitalWrite(LED1, HIGH);
      XBee.println("LED1 is on");
      continue;
    }
    else if (buffer == '2'){
      digitalWrite(LED2, HIGH);
      XBee.println("LED2 is on");
      continue;
    }
    else if (buffer == '3'){
      digitalWrite(LED3, HIGH);
      XBee.println("LED3 is on");
      continue;
    }
    else if (buffer == '4'){
      digitalWrite(LED4, HIGH);
      XBee.println("LED4 is on");
      continue;
    }
    else if (buffer == '5'){
      digitalWrite(LED1, LOW);
      XBee.println("LED1 is off");
      continue;
    }
    else if (buffer == '6'){
      digitalWrite(LED2, LOW);
      XBee.println("LED2 is off");
      continue;
    }
    else if (buffer == '7'){
      digitalWrite(LED3, LOW);
      XBee.println("LED3 is off");
      continue;
    }
    else if (buffer == '8'){
      digitalWrite(LED4, LOW);
      XBee.println("LED4 is off");
      continue;
    }
    else if (buffer == 'o'){
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
      XBee.println("LED1 is on");
      XBee.println("LED2 is on");
      XBee.println("LED3 is on");
      XBee.println("LED4 is on");
      continue;
    }
    else if (buffer == 'f'){
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      XBee.println("LED1 is off");
      XBee.println("LED2 is off");
      XBee.println("LED3 is off");
      XBee.println("LED4 is off");
      continue;
    }

}
}
