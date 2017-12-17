#include "data.h"


Data::Data(int id_){
    id=id_;
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
    accE=0;
}

void Data::StoreNewData(float t_,float l_, float d_, bool o_){
    t.push_back(t_);
    l.push_back(l_);
    d.push_back(d_);
    o.push_back(o_); 
    ComputeEnergy();
}

void Data::ComputeEnergy(){
    std::vector<float>::iterator it=t.end();
    E.push_back(d.back()*( *(it-1)-*(it-2) ) );
    accE+=E.back();
}

void Data::SetGains(std::vector<float> k_){
    k=k_;
}

void Data::SetReference(float r_){
    r=r_;
}
		  
float Data::GetIlluminance(){
    return l.back();
}

float Data::GetDutyCycle(){
    return d.back();
}

bool Data::GetOccupancy(){
    return o.back();
}

float Data::GetAccumulatedEnergy(){
    return accE;
}

float Data::GetIlluminanceLowerBound(){
    if(o.back()==FALSE) return 25;
    else return 50;
}

float Data::GetExternalIlluminance(std::vector<Data> arduino){
    float emitted=0;
    int n=k.size();
    for(int i=0;i<n;++i)
      emitted+=k[i]*arduino[i].GetDutyCycle();
    if(l.back()>emitted) return l.back()-emitted;
    else return 0;
}

float Data::GetReference(){
    return r;
}

float Data::GetInstantaneousPower(){
    return E.back();
}

float Data::GetComfortError(){
    int n=t.size()-1;
    float C=0;
    for(int i=1;i<n+1;++i){
      if(r>l[i]) C+=r-l[i];
    }
    C=C/n;
    return C;
}

float Data::GetComfortVariance(){
    int n=t.size()-1;
    std::cout << n << std::endl;
    float V=0;
    float Ts2=0.025*0.025;
    for(int i=3;i<n+1;++i){
      V+=std::abs(l[i]-2*l[i-1]+l[i-2]);
    }
    V=V/n/Ts2;
    return V;
}
