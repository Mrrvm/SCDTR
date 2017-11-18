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
  //first step: arduino 1 calibrates himself, so arduino 2 receives K11
  if(step==0 && my_address==2){
    while(Wire.available() >	0)	{
      int u1=Wire.read();
      int u2=Wire.read();
      K[step]=(double)(u2*256+u1)/10000;
    }
    step++;
    //Serial.println("ola");
  }
  //second step: arduino 2 calibrates himself, so arduino 1 receives K22
  if(step==1 && my_address==1){
    while(Wire.available() >	0)	{
      int u1=Wire.read();
      int u2=Wire.read();
      K[step]=(double)(u2*256+u1)/10000;
    }
    step++;
  }
  //third step: arduino 1 comands arduino 2 to increase pwm to calibrate K21
  if(step==2 && my_address==2){
    while(Wire.available() >	0)	{
      //if pwm reached end, arduino 1 sends the result K21 to arduino 2
      if(count==256){
        int u1=Wire.read();
        int u2=Wire.read();
        K[step]=(double)(u2*256+u1)/10000;
        count=0;
        step++;
      }
      //if not, just keeps increasing the light
      else {
        dcycle	=	Wire.read();
        count++;
      }
    }
    analogWrite(analogOutPin,dcycle);
  //fourth step: arduino 2 comands arduino 1 to increase pwm to calibrate K12
  }
  if(step==3 && my_address==1){
    while(Wire.available() >	0)	{
      //if pwm reached end, arduino 2 sends the result K12 to arduino 1
      if(count==256){
        int u1=Wire.read();
        int u2=Wire.read();
        K[step]=(double)(u2*256+u1)/10000;
        count=0;
        step++;
      }
      //if not, just keeps increasing the light
      else {
        dcycle	=	Wire.read();
        count++;
      }
    }
    analogWrite(analogOutPin,dcycle);
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
	   analogWrite(analogOutPin,i); //writes duty cycle in it's OWN LED
	   delay(30);
	   sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	   sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	   sensorLux=lux_converter(sensorVoltage);
	   Serial.print(i);
	   Serial.print(": ");
	   Serial.println(sensorLux);
	   if(i>50) //ignores first values because of non linearity
	    sum+=sensorLux/i;
  }
  analogWrite(analogOutPin,0);
  K[step]=sum/205; //median value of gains for each pwm
  Serial.print("K: ");
	Serial.println(K[step]);
  int u=(int)(K[step]*10000); //so converto em integer porque acho mais facil enviar inteiros 
                              //deve haver maneiras mais inteligentes
  Wire.beginTransmission(0);
  Wire.write(lowByte(u));     //integer=2 bytes, so 1 de cada vez pode ser enviado
  Wire.write(highByte(u));
  Wire.endTransmission();
  step++;
}

void othercalib(){
  analogWrite(analogOutPin,0);
  double sum=0;
  for(int i=0;i<=255;++i){
    Wire.beginTransmission(0); //master arduino sends to slave the duty cycle he wants on OTHER'S LED
    Wire.write(i);
    Wire.endTransmission(); 
    delay(30);
    sensorValue=analogRead(analogInPin); //then reads it's OWN value of lux
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
  int u=(int)(K[step]*10000); //same as before
  Serial.print("u: ");
	Serial.println(u);
  Wire.beginTransmission(0);
  Wire.write(lowByte(u));
  Wire.write(highByte(u));
  Wire.endTransmission();
  step++;
}

void loop() {
  
  //basicamente aqui e quem toma a iniciativa de ser master em cada step
  if(my_address==1 && step==0) //step=0 => arduino 1 toma as redeas e faz self calib
    selfcalib();
  if(my_address==2 && step==1) //step=1 => arduino 2 toma as redeas e faz self calib
    selfcalib();
  delay(50);
  if(my_address==1 && step==2) //step=2 => arduino 1 toma as redeas e faz other calib
    othercalib();
  if(my_address==2 && step==3) //step=3 => arduino 2 toma as redeas e faz other calib
    othercalib();
  
  //quando termina a calibracao, ambos imprimim no serial os 4 ganhos
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
  

}