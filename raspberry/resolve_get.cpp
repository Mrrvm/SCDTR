#include "resolve_get.h"

extern std::vector<Data> inoData;

std::string resolve_get(std::string comm) {
  
  std::string sol;
  std::ostringstream ss;
  char c2=comm.at(0);
  int space = comm.find_last_of(" ");
  std::string index=comm.substr(space+1);
  // g l <i>
  if(c2 == 108) {
    float val = 0;
    int c3=stoi(index);
    val = inoData[c3].GetIlluminance();
    ss << c2 << " " << c3 << val;
    sol=ss.str();
  }
  // g d <i>
  if(c2 == 100) {
    float val = 0;
    int c3=stoi(index);
    val = inoData[c3].GetDutyCycle();
    ss << c2 << " " << c3 << val;
    sol=ss.str();
  }
  // g o <i>
  if(c2 == 111) {
    bool val= 0;
    int c3=stoi(index);
    val = inoData[c3].GetOccupancy();
    ss << c2 << " " << c3 << val;
    sol=ss.str();
  }
  // g L <i>
  if(c2 == 76) {
    float val = 0;
    int c3=stoi(index);
    val = inoData[c3].GetIlluminanceLowerBound();
    ss << c2 << " " << c3 << val;
    sol=ss.str();
  }
  // g O <i>
  if(c2 == 79) {
    float val = 0;
    int c3=stoi(index);
    val = inoData[c3].GetExternalIlluminance();
    ss << c2 << " " << c3 << val;
    sol=ss.str();
  }
  // g r <i>
  if(c2 == 114) {
    float val = 0;
    int c3=stoi(index);
    val = inoData[c3].GetReference();
    ss << c2 << " " << c3 << val;
    sol=ss.str();
  } 
  if(c2 == 79) {
    // g p T
    if(index.at(0) == 84) {
      // POR AQUI CENAS
      float val=0;
      for(int i=0;i<N_ino;++i)
	val+=inoData[i].GetInstantaneousPower();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
    // g p <i>
    else {
      float val = 0;
      int c3=stoi(index);
      val = inoData[c3].GetInstantaneousPower();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
  }
  if(c2 == 101) {
    // g e T
    if(index.at(0) == 84) {
      float val=0;
      for(int i=0;i<N_ino;++i)
	val+=inoData[i].GetAccumulatedEnergy();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
    // g e <i>
    else {
      float val = 0;
      int c3=stoi(index);
      val = inoData[c3].GetAccumulatedEnergy();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
  } 
  if(c2 == 99) {
    // g c T
    if(index.at(0) == 84) {
      // POR CENAS AQUI
      float val=0;
      for(int i=0;i<N_ino;++i)
	val+=inoData[i].GetComfortError();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
    // g c <i>
    else {
      float val = 0;
      int c3=stoi(index);
      val = inoData[c3].GetComfortError();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
  } 
  if(c2 == 118) {
    // g v T
    if(index.at(0) == 84) {
      // POR CENAS AQUI
      float val=0;
      for(int i=0;i<N_ino;++i)
	val+=inoData[i].GetComfortVariance();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
    // g v <i>
    else {
      float val = 0;
      int c3=stoi(index);
      val = inoData[c3].GetComfortVariance();
      ss << c2 << " " << c3 << val;
      sol=ss.str();
    }
  } 
  return sol;
}
