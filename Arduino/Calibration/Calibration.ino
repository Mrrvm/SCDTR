#include <Wire.h>
#include <EEPROM.h>

const int analogInPin = A0;
const int analogOutPin = 9;
const double a[2]={-0.61092,-0.61092}; //{-0.61092,-0.57092} for equal gains
const double b=4.69897;
int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;
int stop=0, count=0; 
unsigned int dcycle=0;

double K[4]={0};
int my_address;
int step=0;
double store[255]={0};


void receiveEvent(int howMany)	{
  if(step==0 && my_address==2){
    while(Wire.available() >	0)	{
      int u1=Wire.read();
      int u2=Wire.read();
      K[step]=(double)(u2*256+u1)/10000;
    }
    step++;
    //Serial.println("ola");
  }
  if(step==1 && my_address==1){
    while(Wire.available() >	0)	{
      int u1=Wire.read();
      int u2=Wire.read();
      K[step]=(double)(u2*256+u1)/10000;
    }
    step++;
  }
  if(step==2 && my_address==2){
    while(Wire.available() >	0)	{
      if(count==256){
        int u1=Wire.read();
        int u2=Wire.read();
        K[step]=(double)(u2*256+u1)/10000;
        count=0;
        step++;
      }
      else {
        dcycle	=	Wire.read();
        count++;//	receive	byte	as	a	character
      }
    }
    //Serial.print("DC: ");
    //Serial.println(dcycle);
    //Serial.print("cnt: ");
    //Serial.println(count);
    analogWrite(analogOutPin,dcycle);
    //if(dcycle==255) step++;
  }
  if(step==3 && my_address==1){
    while(Wire.available() >	0)	{
      if(count==256){
        int u1=Wire.read();
        int u2=Wire.read();
        K[step]=(double)(u2*256+u1)/10000;
        count=0;
        step++;
      }
      else {
        dcycle	=	Wire.read();
        count++;//	receive	byte	as	a	character
      }
    }
    //Serial.print("DC: ");
    //Serial.println(dcycle);
    //Serial.print("cnt: ");
    //Serial.println(count);
    analogWrite(analogOutPin,dcycle);
    //if(dcycle==255) step++;
  }
}

void setup() {
  Serial.begin(9600); 
  my_address=EEPROM.read(0);
  Wire.begin(my_address);
  TWAR = (my_address << 1) | 1;
  Wire.onReceive(receiveEvent);
}

double lux_converter(double volt){
  double l=pow(((5/volt)-1)*10000/pow(10,b),1/a[my_address-1]);
  return l;
}

double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max){
  return (double)(vi-vi_min)*(vo_max-vo_min)/(double)(vi_max-vi_min)+vo_min;
}

void selfcalib(){
  double sum=0;
  for(int i=0;i<=255;i=i+1){   
	   analogWrite(analogOutPin,i);
	   delay(30);
	   sensorValue=analogRead(analogInPin);
	   sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	   sensorLux=lux_converter(sensorVoltage);
	   Serial.print(i);
	   Serial.print(": ");
	   Serial.println(sensorLux);
	   if(i>85)
	    sum+=sensorLux/i;
  }
  analogWrite(analogOutPin,0);
  K[step]=sum/170;
  Serial.print("K: ");
	Serial.println(K[step]);
  int u=(int)(K[step]*10000);
  //Serial.println("ola2");
  Wire.beginTransmission(0);
  Wire.write(lowByte(u));
  Wire.write(highByte(u));
  Wire.endTransmission();
  step++;
}

void othercalib(){
  //Serial.println(my_address);
  //Serial.println(step);
  analogWrite(analogOutPin,0);
  double sum=0;
  for(int i=0;i<=255;++i){
    Wire.beginTransmission(0);
    Wire.write(i);
    Wire.endTransmission(); 
    delay(30);
    sensorValue=analogRead(analogInPin);
	  sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	  sensorLux=lux_converter(sensorVoltage);
	  Serial.print(i);
	  Serial.print(": ");
	  Serial.println(sensorLux);
	  if(i>25)
      sum+=sensorLux/i;
  }
  K[step]=sum/230;
  Serial.print("K: ");
	Serial.println(K[step]);
  int u=(int)(K[step]*10000);
  Serial.print("u: ");
	Serial.println(u);
  //Serial.println("ola2");
  Wire.beginTransmission(0);
  Wire.write(lowByte(u));
  Wire.write(highByte(u));
  Wire.endTransmission();
  step++;
}

void loop() {
  
  if(my_address==1 && step==0)
    selfcalib();
  if(my_address==2 && step==1)
    selfcalib();
  delay(50);
  if(my_address==1 && step==2)
    othercalib();
  if(my_address==2 && step==3)
    othercalib();
  if(step==4){
    Serial.print(K[0]);
    Serial.print(" ");
    Serial.print(K[1]);
    Serial.print(" ");
    Serial.print(K[2]);
    Serial.print(" ");
    Serial.println(K[3]);
  }
  delay(10);
  
/*  if(stop==0){
  	for(int i=0;i<=250;i=i+1){   
	   analogWrite(analogOutPin,i);
	   delay(50);
	   sensorValue=analogRead(analogInPin);
	   sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	   sensorLux=lux_converter(sensorVoltage);
	   Serial.println(sensorLux);
  	}
  	stop=1;
  }
  delay(10);*/
}