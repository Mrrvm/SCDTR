#include <Wire.h>


void receiveEvent(int howMany)	{
  while(Wire.available() >	0)	{
    char	c	=	Wire.read();	//	receive	byte	as	a	character
    Serial.write(c);	//	echo
  }
}

void setup() {
  Serial.begin(9600);  
  Wire.begin(1);
  Wire.onReceive(receiveEvent);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}


void loop() {
  char c;
  if(Serial.available()>0)	{
    c=Serial.read();
    Wire.beginTransmission(0);	
    Wire.write(c);	
    Wire.endTransmission();
  }

}
