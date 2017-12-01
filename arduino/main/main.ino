#include "PID.h"

const int analogInPin = A0;
const int analogOutPin = 9;

int sensorValue=0;
double sensorVoltage=0;
double sensorLux=0;

int setpoint_changes=0;

unsigned long sample=25;
double Kp=0.1, Ki=0.1, Kd=0, pole_value=10;

bool fftoggle = 1;

PID myPID;

void setup() {
  Serial.begin(115200);
  myPID.Reset();
  myPID.SetReference(HIGH);
  myPID.SetSamplingTime(sample);
  myPID.SetParameters(Kp,Ki,Kd,pole_value);
  pinMode(13, OUTPUT);
}


void loop() {
  unsigned long startTime=millis(); 
  //time at start of loop
  digitalWrite(13, HIGH); 
  sensorValue = analogRead(analogInPin);                    //input in 0-1023 range
  sensorVoltage=map_double(sensorValue,0,1023,0,5); //maps input to voltage interval 0-5
  sensorLux=lux_converter(sensorVoltage);             //converts to lux
  myPID.Control(sensorLux, analogOutPin);                           //computes with input in lux
  String message = String(myPID.GetReference());
  message += (" "+String(sensorLux));
  message += (" "+String(myPID.GetPWMPercent()));
  message += (" "+String(fftoggle));
  message += (" "+String(millis()));
  Serial.println(message);

  unsigned long endTime=millis();                     //time at end of loop
  delay(sample-(endTime-startTime));                  //delay to avoid over computation of "control"
  
  while (Serial.available()) {
    int read=Serial.parseInt();
    if(read>0 && read<70) {
      Serial.print("Nova Referência: ");
      Serial.println(read);
      myPID.SetReference(read);
    }
    else if(read == -1) {
      fftoggle = myPID.ToggleFeedforward();
      Serial.print("Feedforward status: ");
      Serial.println(fftoggle);
    }
  }  
}

