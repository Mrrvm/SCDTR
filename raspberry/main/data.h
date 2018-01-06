#ifndef     __DATA_H__
#define     __DATA_H__

#include <vector>
#include <iostream>
#include <cmath>
#include "defs.h"

class Data{

private:

  //id
  int id;
  
  // last restart index
  int last_r;
  
  //data
  std::vector <int> t;
  std::vector <float> l;
  std::vector <float> d;
  std::vector <float> o;

  //calculations
  std::vector <float> E;

  //special data
  float r;
  float external_illuminance;

  
public:
  Data(int);

  void StoreNewData(int,float, float, bool);
  void ComputeEnergy();

  void SetReference(float);

  void SetExternalIlluminance(float);

  void SetRestartTime();

  int GetTimestamp();
      
  float GetIlluminance();

  float GetDutyCycle();

  bool GetOccupancy();

  float GetAccumulatedEnergy();

  float GetIlluminanceLowerBound();

  float GetExternalIlluminance();

  float GetReference();

  float GetInstantaneousPower();

  float GetComfortError();

  float GetComfortVariance();
  
  std::string GetLastMinuteBuffer(bool);


  
};



#endif
