// Reading from HMC6352 and send it to serial
#include <Wire.h>     // need to the I2C Protocol used by the HMC6352
 
void setup(){
  Serial.begin(9600); // Initiate Serial
  Wire.begin();       // Initiate I2C, No param means "join as master" 
}
 
void loop(){
  Wire.beginTransmission(0x21);
  Wire.write("A");     // Send "Get Data" command, info from Datasheet
  delay(100);         // interface command delay, info from Datasheet
  Wire.requestFrom(0x21, 2); //get the two data bytes, MSB and LSB
  byte MSB = Wire.read(); // Result will be in tenths of degrees (0 to 3599)
  byte LSB = Wire.read(); // Provided in binary format over two bytes."
  Wire.endTransmission();
  // Compute result from the two bytes results
  float myres = ((MSB << 8) + LSB) / 10; 
  // Display result
  Serial.print(myres);
  Serial.println(" degrees");
  //delay(10);
}
