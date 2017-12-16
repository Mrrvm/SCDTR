#ifndef     __DATA_H__
#define     __DATA_H__

#include <vector>
#include <iostream>
#include <cmath>

class Data{

private:

  //id
  int id;
  
  
  //data
  std::vector <float> t;
  std::vector <float> l;
  std::vector <float> d;
  std::vector <float> o;

  //calculations
  std::vector <float> E;
  float accE; 

  //special data
  float r;
  float external_illuminance;
  std::vector<float> k;

  
public:
  Data(int);

  void StoreNewData(float,float, float, float);
  void ComputeEnergy();

  void SetGains(std::vector<float>);

  void SetReference(float r_);
		  
  float GetIlluminance();

  float GetDutyCycle();

  float GetOccupancy();

  float GetAccumulatedEnergy();

  float GetIlluminanceLowerBound();

  float GetExternalIlluminance(std::vector<Data>);

  float GetReference();

  float GetInstantaneousPower();

  float GetComfortError();

  float GetComfortVariance();
  
};



#endif
