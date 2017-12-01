#include <Wire.h>
#include <EEPROM.h>

#define N 2
#define M 3

const int analogInPin = A0;
const int analogOutPin = 9;
const double a[2]={-0.61092,-0.61092}; //{-0.61092,-0.57092} for equal gains
const double b=4.69897;
int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;
int count_sum=0, count=0; 
unsigned int dcycle=0;
double other_sum=0;

double K[N][N]={0};
int my_address;
int step=1;
int step2=1;


void receiveEvent(int howMany)	{
  //Serial.println("ENVIOU");
  if(step!=my_address && step<=N){
    while(Wire.available() >	0)	{
      dcycle=Wire.read();
    sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	  sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	  sensorLux=lux_converter(sensorVoltage);
	  if(dcycle>50){ 
	    other_sum+=sensorLux/dcycle;
	    count_sum++;
	  }
	  count++;
	  //Serial.println(count);
	  //Serial.print("D: ");
	  //Serial.println(dcycle);
	  if(dcycle==256-1 && count>(M-1)*256+100){
	    K[step-1][my_address-1]=other_sum/count_sum;
	    count=0;
	    count_sum=0;
	    other_sum=0;
	    step++;
	    //Serial.print("Step: ");
	    //Serial.println(step);
	    //analogWrite(analogOutPin,255);
	  }
    }
  }
  //Serial.println(howMany);
  if(step>N && step2!=my_address && step2<=N){
    //Serial.println("READS!!");
    while(Wire.available()>0){
      //Serial.println("READS!!");
      int r=Wire.read();
      int u1=Wire.read();
      int u2=Wire.read();
      //Serial.println(r);
      K[r-1][step2-1]=(double)(u2*256+u1)/1000;
      //Serial.println(u1);
      //Serial.println(u2);
      if(r==N) step2++;
      //Serial.println(step2);
    }
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


void calib(){
  //Serial.println("OLA");
  double self_sum=0;
  for(int j=0;j<M;++j){
    for(int i=0;i<=255;++i){
      analogWrite(analogOutPin,i);
      delay(30);
      sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	    sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	    sensorLux=lux_converter(sensorVoltage);
	    if(i>50) //ignores first values because of non linearity
	      self_sum+=sensorLux/i;
	    Wire.beginTransmission(0); //master arduino sends to slave the duty cycle he wants on OTHER'S LED
      Wire.write(i);
      Wire.endTransmission();
      //Serial.println(i+256*j);
    }
  }
  analogWrite(analogOutPin,0);
  K[step-1][my_address-1]=self_sum/205/M;
  /*Serial.print(my_address);
  Serial.print(" ");
  Serial.print(step);
  Serial.print(" ");
  Serial.println(K[step-1][my_address-1]);*/
  step++;
}


void loop() {
  //Serial.print("Step: ");
  //Serial.println(step);
  //Serial.print("Add: ");
  //Serial.println(my_address);
  while(step<=N){
    delay(5);
    //Serial.print("Step: ");
    //Serial.println(step);
    //Serial.print("Address: ");
    //Serial.println(my_address);
    if(step==my_address){
      calib();
    }
  }
  //step2=1;
  while(step2<=N){
    delay(5);
    //Serial.println("BLOCKXXX ");
    //Serial.println(step2);
    if(step2==my_address){
      //Serial.println("BLOCK");
      for(int h=1;h<=N;++h){
        //Serial.println("BLOCK");
        int u=(int)(K[h-1][my_address-1]*1000); //same as before
        Wire.beginTransmission(0);
        Wire.write(h);
        Wire.write(lowByte(u));
        Wire.write(highByte(u));
        Wire.endTransmission();
        //Serial.println("BLOCK");
      }
    step2++;
    }
  }
  if(step>N && step2>N){
    Serial.print(K[0][0]);
    Serial.print(" ");
    Serial.println(K[0][1]);
    Serial.print(K[1][0]);
    Serial.print(" ");
    Serial.println(K[1][1]);
    Serial.println(" ");
    delay(100);
  }
  
  

}