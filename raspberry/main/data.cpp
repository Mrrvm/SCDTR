/** data.cpp 
 *
 *  Summary:     Saves and makes calculations with the retrieved information.
 *  Last Edited: December 29, 2017
 *  Authors:     Mariana Martins (mrrvm@hotmail.com)
 *               Filipe Madeira  (filipe.s.c.madeira@gmail.com)
 *               Carlos Aleluia  (carlos.aleluia@tecnico.ulisboa.pt)
 *  License:     GNU General Public License v3.0
 *
 */

#include "data.h"

Data::Data(int id_){
  id=id_;
  last_r=1;
  t.reserve(1000);
  l.reserve(1000);
  d.reserve(1000);
  o.reserve(1000);
  E.reserve(1000);
  t.push_back(0);
  l.push_back(0);
  d.push_back(0);
  o.push_back(0);
  E.push_back(0);
}

void Data::StoreNewData(int t_, float l_, float d_, bool o_){
  t.push_back(t_);
  l.push_back(l_);
  d.push_back(d_);
  o.push_back(o_); 
  ComputeEnergy();
}

void Data::ComputeEnergy(){
  std::vector<int>::iterator it=t.end();
  E.push_back((d.back()/255)*(*(it-1)-*(it-2)));
}


void Data::SetExternalIlluminance(float ex_ilu){
    external_illuminance = ex_ilu;
}

void Data::SetReference(float r_){
    r=r_;
}

void Data::SetRestartTime(){
    last_r=t.size();
}

int Data::GetTimestamp() {
  return t.back()-t[last_r];
}

float Data::GetIlluminance(){
  return l.back();
}

float Data::GetDutyCycle(){
  return d.back()*100/255;
}

bool Data::GetOccupancy(){
  return o.back();
}

float Data::GetAccumulatedEnergy(){
  float accE=0;
  int n=t.size()-last_r;
  for(int i=0;i<n;++i)
    accE+=E[i+last_r];
  return accE;
}

float Data::GetIlluminanceLowerBound(){
    if(o.back()==0) return 25;
    else return 50;
}

float Data::GetExternalIlluminance(){
    return external_illuminance;
}

float Data::GetReference(){
  return r;
}

float Data::GetInstantaneousPower(){
  return d.back()/255;
}


float Data::GetComfortError(){
  int n=t.size()-last_r;
  float C=0;
  for(int i=0;i<n;++i){
    if(r>l[i+last_r]) C+=r-l[i+last_r];
  }
  C=C/n;
  return C;
}

float Data::GetComfortVariance(){
  int n=t.size()-last_r;
  std::cout << n << std::endl;
  float V=0;
  float Ts2=0.025*0.025;
  for(int i=3;i<n;++i){
    V+=std::abs(l[i+last_r]-2*l[i-1+last_r]+l[i-2-last_r]);
  }
  V=V/n/Ts2;
  return V;
}

std::string Data::GetLastMinuteBuffer(bool variable){ //variable: 0-l  1-d
  int ind=t.size();
  std::ostringstream ss;
  if(t.back()-t.front()<60) {
    ss << "-";
    return ss.str();
  }
  if(!variable){ ss << l[ind];}
  else {ss << d[ind];}
  for(ind = t.size()-1 ; (t.back()-t[ind])<60; --ind){
      if(!variable) {ss << "," <<  l[ind];}
      else {ss << "," << d[ind]/255*100;}
    }
  return ss.str();
}

