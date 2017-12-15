#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "Defs.h"
#include <Wire.h>



//PID class

class PID
{
  public:
      PID();  //constructor
      ~PID(); //destructor
      bool Control(double);  //control loop function
  
  
  
      void SetReference();                           //set reference of PID
      void SetParameters(double, double, double, double);  //set parameters kp,ki,kd,pole
      void SetSamplingTime(unsigned long);                 //set sampling time
      void Reset();                                        //resets all members to default (constructor) values
      bool ToggleFeedforward();
      void SetOccupancy(bool);
      double GetReference();
      double GetPWMPercent();
      int GetFullResponse();
      bool GetOccupancy();
  
  private:
  
      double Kp;
      double Ki;
      double Kd;
      double Pole;

      double K2;
      double K3;
      double K4;
      
      double reference;
      int feedforward;
      int full_response;
      bool reference_change;   //whether the reference has been changed after the last computation

      double Previous_input;        //input at last computation
      double I;                     //integral term of PID
      double D;                     //derivative term of PID
      double Previous_error;        //error at last computation
      
      unsigned long Previous_Time;  //instant of last computation
      unsigned long Sampling_Time;  //sampling time
  
      bool feedforward_on;
      bool occupancy;
  
};

#endif