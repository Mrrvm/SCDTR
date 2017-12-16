#include "PID.h"


float consensus(float L);

//Constructor
PID::PID(){
  
  //set all members to default parameters
  Previous_input=0;
  I=0;
  D=0;
  Previous_error=0;
  Previous_Time=millis()-Sampling_Time;
  feedforward_on = 1;
  occupancy=1;
  
  reference_change=false;

}

//Destructor
PID::~PID(){
  
}

//Control loop function
bool PID::Control(double input){ 
  
  //check if time interval between last computation
  //and present one is at least one sample interval
  unsigned long Present_Time = millis();
  unsigned long deltaT = (Present_Time - Previous_Time);
  
  //initialize aux variables as zero
  double error=0;
  double u=0;
  
  if(deltaT>=Sampling_Time)
   {
      //checks if has to include feedback term
      if(!reference_change || !feedforward_on){
        error=reference-input;
        
        //DEADZONE
        if(error >= ERROR_MAX) {
          error = error - ERROR_MAX;
        }
        else if(error <= -ERROR_MAX) {
          error = error - (-ERROR_MAX);
        }
        else
          error = 0;
          

        //Proportional
        double P=Kp*error;
        
        //Integral
        I+=K2*(error+Previous_error);
        
        //ANTI-WINDUP
        if(I>W) {
          I=W;
        }
        else if (I<-W){
          I=-W;
        }
        
        //Derivative
        D*=K3;
        D-=K4*(input-Previous_input);
        
        //Total feedback=PID (in lux)
        u=0;//P+I+D;

      }
      
      //makes variable false so in next computation feedback is included
      else
        reference_change=false;

    int feedback=u/M;//K[my_address-1];  //total feedback (in PWM)
    if(feedforward_on) {
      full_response=feedback+feedforward;
    }
    else { 
      full_response = feedback;
    }

    //Check maximum values of PWM
    if(full_response>255) full_response=255;
    else if(full_response<0) full_response=0;
    analogWrite(analogOutPin, full_response);

    //Records variables of present calculation
    Previous_error=error;
    Previous_input=input;
    Previous_Time=Present_Time;
    return true;      
   }
  else return false;  //interval was smaller than sampling time
}

void PID::SetOccupancy(bool occ){
  if(occ!=occupancy){
    Wire.beginTransmission(0); 
	  Wire.write("z");
    Wire.endTransmission();
    //Serial.println("trocou ocupacao");
    occupancy=occ;
    SetReference();
  }
}


void PID::SetReference(){
  //Serial.println("Setting reference...");
  if(occupancy) reference=HIGH; //50
  else reference=LOW;           //25
  //Serial.println(reference);
  float gain=consensus(reference);
  //Serial.println(gain);
  feedforward=gain;
  reference_change=true;       //in next cycle, feedback will not be included
}

void PID::SetParameters(double kp_value, double ki_value, double kd_value, double pole_value){
  Kp=kp_value;
  Ki=ki_value;
  Kd=kd_value;
  Pole=pole_value;
  
  K2=Kp*Ki*Sampling_Time/2;
  K3=Kd/(Kd+Pole*Sampling_Time);
  K4=Kp*Kd*Pole/(Kd+Pole*Sampling_Time);
}

void PID::SetSamplingTime(unsigned long Sampling_Time_value){
  Sampling_Time=Sampling_Time_value;
}

void PID::Reset(){
  Previous_input=0;
  I=0;
  D=0;
  Previous_error=0;
  Previous_Time=millis()-Sampling_Time;
  
  reference_change=false;
}


bool PID::ToggleFeedforward(){
  feedforward_on = !feedforward_on;
  return feedforward_on;
}

double PID::GetReference(){
  return reference;
}

double PID::GetPWMPercent(){
  return ((double)full_response/255)*100;
}

int PID::GetFullResponse(){
  return full_response;
}

bool PID::GetOccupancy(){
  return occupancy;
}
