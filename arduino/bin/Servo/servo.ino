// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
int servoPin = 9; 
int vddPin=13;
// Create a servo object 
Servo Servo1; 
void setup() { 
  Serial.begin(115200);
   // We need to attach the servo to the used pin number 
   Servo1.attach(servoPin); 
   digitalWrite(vddPin,HIGH);
}
void loop(){ 
  /*Servo1.write(0);
  delay(1000);
  Servo1.write(180);
  delay(1000);*/
  while (Serial.available()) {
    byte serial_read=Serial.parseInt();
    if(serial_read==1){
      Servo1.write(0); 
      delay(1000); 
    }
    if(serial_read==2){
      Servo1.write(180); 
      delay(1000); 
    }
  }
}  