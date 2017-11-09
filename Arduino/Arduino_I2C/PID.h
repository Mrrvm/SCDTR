#ifndef PID_h
#define PID_h

class PID
{
  public:
      PID(double*, double*, double, double, double, double, int);
      bool Control();
  
  
  
      void SetReference(double);
      void SetParameters(double, double, double);
      void SetSamplingTime(int);
      double GetKp();
      double GetKi();
      double GetKd();
  
  private:
  
      double Kp;
      double Ki;
      double Kd;
      
      double *PID_input;
      double *PID_output;
      double PID_reference;
      
      unsigned long Previous_Time;
      unsigned long Sampling_Time;
  
  
};


#endif