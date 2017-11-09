#include "PID.h"
#include "Arduino.h"

PID::PID(double* input, double* output, double reference_value, double kp_value, 
          double ki_value, double kd_value, int Sampling_Time_value){
  
  PID_input=input;
  PID_output=output;
  
  PID::SetReference(reference_value);
  
  PID::SetParameters(kp_value, ki_value, kd_value);
  
  PID::SetSamplingTime(Sampling_Time_value);
  
  Previous_Time=millis()-Sampling_Time;

}


bool PID::Control(){ return true;}

void PID::SetReference(double reference_value){
  PID_reference=reference_value;
}

void PID::SetParameters(double kp_value, double ki_value, double kd_value){
  Kp=kp_value;
  Ki=ki_value;
  Kd=kd_value;
}

void PID::SetSamplingTime(int Sampling_Time_value){
  Sampling_Time=Sampling_Time_value;
}

double PID::GetKp(){ return Kp;}
double PID::GetKi(){ return Ki;}
double PID::GetKd(){ return Kd;}

