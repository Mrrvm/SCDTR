#include "PID.h"
#include "Defs.h"


int my_address=0;
bool calibrated=0;
int count_sum=0, count=0; 
double other_sum=0;
//float K[N]={0};
//float o=0;
//float d[N][N]={0};
bool beginning=1;
byte window=0;

byte step=1;
byte step2=1;


//int sensorValue=0;
//double sensorVoltage=0;
//double sensorLux=0;


unsigned long sample=25;
double Kp=0.1, Ki=0.1, Kd=0, pole_value=10;

bool fftoggle = 1;

PID myPID;
Servo Servo1; 

bool call_consensus=0;
bool change_occupancy[2]={0};
bool change_consensus[2]={0};

bool loop_calib(){
  analogWrite(analogOutPin,0);
  delay(1000);
  myPID.o=0;
  for(int j=0;j<C;++j){
    int sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	  double sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	  myPID.o+=lux_converter(sensorVoltage);
  }
  myPID.o=myPID.o/C;
  unsigned long start_while=millis();
  while(step<=N){
      delay(5);
      if(step==my_address){
        calib();
      }
      
      if(millis()-start_while>30000+5000*C*N){ //detect if arduino is dead
        myPID.K[step-1]=0;
        step++;
        start_while=millis();
      }
    }
  step=1;
  count_sum=0;
  count=0;
  other_sum=0;
  return true;
}


void calib(){
  double self_sum=0;
  for(int j=0;j<C;++j){
    for(int i=0;i<=255;++i){
      analogWrite(analogOutPin,i);
      delay(30);
      int sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	    double sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	    double sensorLux=lux_converter(sensorVoltage);
	    if(i>50) //ignores first values because of non linearity
	      self_sum+=(sensorLux-myPID.o)/i;
	   //Serial.println(sensorLux);
	    Wire.beginTransmission(0); //master arduino sends to slave the duty cycle he wants on OTHER'S LED
	    Wire.write("k");
      Wire.write(i);
      Wire.endTransmission();
    }
  }
  analogWrite(analogOutPin,0);
  myPID.K[step-1]=self_sum/205/C;
  step++;
}


void receiveEvent(int howMany)	{
    char phase=0;
    int h=0;
    int b=0;
    while(Wire.available() >	0)	{

    //Serial.println("recebi");
    if(howMany>0) phase=Wire.read();
    if(howMany>1) h=Wire.read();
    if(howMany>2) b=Wire.read();}
    //Serial.println(phase);
    //if(howMany>3) break;
    //Serial.println("passou");
    //Serial.println(phase);
    //Serial.println(h);
    //Serial.println(b);
    if(phase==107){
      if(step!=my_address && step<=N){
        //dcycle=Wire.read();
        int sensorValue=analogRead(analogInPin); //then reads in it's OWN LDR
	      double sensorVoltage=map_double((double)sensorValue,0,1023,0,5);
	      double sensorLux=lux_converter(sensorVoltage);
	      if(h>50){ 
	        other_sum+=(sensorLux-myPID.o)/h;
	        count_sum++;
	      }
	      count++;
	      if(h==256-1 && count>(C-1)*256+100){
	        myPID.K[step-1]=other_sum/count_sum;
	        count=0;
	        count_sum=0;
	        other_sum=0;
	        step++;
	      }
      }
    }
    if(phase==99){
      myPID.d[h][step2-1]=b;
      if(h==N-1) step2++;
    }
    if(phase==122){
      Serial.println("vai comecar o consensus");
      call_consensus=1;
    }
    if(phase==114){
      Serial.println("vai comecar a calib");
      calibrated=0;
    }
    if(phase==112){
      change_consensus[1]=(bool)h; 
      change_consensus[0]=1;
    }
    if(phase==115){
      if(h==(my_address-1)*2+1) {change_occupancy[1]=0; change_occupancy[0]=1;}
      if(h==(my_address-1)*2+2) {change_occupancy[1]=1; change_occupancy[0]=1;}
    }
    if(phase==98 && beginning==0){
      Serial.println("vai comecar a recalib");
      calibrated=0;
    }
    if(phase==119){
      Serial.print("vai ligar o motor ");
      Serial.println(h);
      if(window!=h){
          Servo1.write(h); 
          calibrated=0;
          window=h;
        }
    }
  //}
}



void setup() {
  Serial.begin(115200);
  Servo1.attach(servoPin);
  myPID.Reset();
  myPID.SetSamplingTime(sample);
  myPID.SetParameters(Kp,Ki,Kd,pole_value);
  pinMode(13, OUTPUT);
  my_address=EEPROM.read(0);
  Wire.begin(my_address);
  TWAR = (my_address << 1) | 1;
  Wire.onReceive(receiveEvent);
  Wire.beginTransmission(0);
  Wire.write("b");
  Wire.endTransmission();
  beginning=0;
}

void loop() {
  if(!calibrated){
    calibrated=loop_calib();
    myPID.SetReference();
    Serial.println(myPID.o);
    Serial.println(myPID.K[0],5);
    Serial.println(myPID.K[1],5);
  }
  
  bool sendData=0;

  unsigned long startTime=millis(); 
  //time at start of loop
  digitalWrite(13, HIGH);
  int sensorValue=0;
  for(int j=0;j<MEASURES;++j)
    sensorValue += analogRead(analogInPin);                    //input in 0-1023 range
  sensorValue=sensorValue/MEASURES;  
  double sensorVoltage=map_double(sensorValue,0,1023,0,5); //maps input to voltage interval 0-5
  double sensorLux=lux_converter(sensorVoltage);             //converts to lux
  sendData=myPID.Control(sensorLux);  //computes with input in lux
  //Serial.println(sensorLux);
  /*String message = String(myPID.GetReference());
  message += (" "+String(sensorLux));
  message += (" "+String(myPID.GetPWMPercent()));
  message += (" "+String(fftoggle));
  message += (" "+String(millis()));
  Serial.println(message);*/
  
  if(sendData){
  byte li=(byte)sensorLux;
  byte di=(byte)(myPID.GetFullResponse());
  byte occi=(byte)(myPID.GetOccupancy());
  Wire.beginTransmission(0);
  Wire.write("a");
  Wire.write(my_address);
  Wire.write("l");
  Wire.write(li);
  Wire.write("d");
  Wire.write(di);
  Wire.write("o");
  Wire.write(occi);
  Wire.endTransmission();}

  unsigned long endTime=millis();                     //time at end of loop
  delay(sample-(endTime-startTime));                  //delay to avoid over computation of "control"
  
  while (Serial.available()) {
    byte serial_read=Serial.parseInt();
    if(serial_read!=0){
      if(serial_read==(my_address-1)*2+1) {change_occupancy[1]=0; change_occupancy[0]=1;}
      if(serial_read==(my_address-1)*2+2) {change_occupancy[1]=1; change_occupancy[0]=1;}
      Serial.println(serial_read);
      if(serial_read<N*2+1){
        Wire.beginTransmission(0);
        Wire.write("s");
        Wire.write(serial_read);
        Wire.endTransmission();
      }
      if(serial_read==N*2+1){ 
        //Serial.println("aqui");
        Wire.beginTransmission(0);
        Wire.write("r");
        Wire.endTransmission();
        calibrated=0;
      }
      if(serial_read==N*2+2) {
        Wire.beginTransmission(0);
        Wire.write("p");
        Wire.write(0);
        Wire.endTransmission();
        Serial.println("vai por consensus em");
        Serial.println("0");
        change_consensus[1]=0; 
        change_consensus[0]=1;
      }
      if(serial_read==N*2+3) {
        Wire.beginTransmission(0);
        Wire.write("p");
        Wire.write(1);
        Wire.endTransmission();
        Serial.println("vai por consensus em");
        Serial.println("1");
        change_consensus[1]=1; 
        change_consensus[0]=1;
      }
      if(serial_read==N*2+4) {
        Wire.beginTransmission(0);
        Wire.write("w");
        Wire.write(0);
        Wire.endTransmission();
        Serial.println("vai ligar o motor 0");
        if(window!=0){
          Servo1.write(0); 
          calibrated=0;
          window=0;
        }
      }
      if(serial_read==N*2+5) {
        Wire.beginTransmission(0);
        Wire.write("w");
        Wire.write(90);
        Wire.endTransmission();
        Serial.println("vai ligar o motor 90");
        if(window!=90){
          Servo1.write(90); 
          calibrated=0;
          window=90;
        }
      }
      if(serial_read==N*2+6) {
        Wire.beginTransmission(0);
        Wire.write("w");
        Wire.write(180);
        Wire.endTransmission();
        Serial.println("vai ligar o motor 180");
        if(window!=180){
          Servo1.write(180); 
          calibrated=0;
          window=180;
        }
      }
    }
  }
  if(call_consensus) {myPID.SetReference(); call_consensus=0;}
  if(change_occupancy[0]) {myPID.SetOccupancy(change_occupancy[1]); change_occupancy[0]=0;}
  if(change_consensus[0]) {myPID.SetConsensus(change_consensus[1]); change_consensus[0]=0;}
  //Serial.flush();
}
