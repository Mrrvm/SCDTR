#include "resolve_get.h"

extern std::vector<Data> inoData;

std::string resolve_get(std::string comm) {
  
  std::string sol;
  std::ostringstream ss;
  char c2 = comm.at(0);
  int space = comm.find_last_of(" ");
  std::string index = comm.substr(space+1);

  // g l <i>
  if(c2 == 108) {
    float val = 0;
    int c3 = std::stoi(index);
    val = inoData[c3-1].GetIlluminance();
    ss << c2 << " " << c3 << " " << val << "\n";
    sol=ss.str();
  }
  // g d <i>
  if(c2 == 100) {
    float val = 0;
    int c3=std::stoi(index);
    val = inoData[c3-1].GetDutyCycle();
    ss << c2 << " " << c3 << " " << val << "\n";
    sol=ss.str();
  }
  // g o <i>
  if(c2 == 111) {
    bool val= 0;
    int c3=std::stoi(index);
    val = inoData[c3-1].GetOccupancy();
    ss << c2 << " " << c3 << " " << val << "\n";
    sol=ss.str();
  }
  // g L <i>
  if(c2 == 76) {
    float val = 0;
    int c3=std::stoi(index);
    val = inoData[c3-1].GetIlluminanceLowerBound();
    ss << c2 << " " << c3 << " " << val << "\n";
    sol=ss.str();
  }
  // g O <i>
  if(c2 == 79) {
    float val = 0;
    int c3=std::stoi(index);
    val = inoData[c3-1].GetExternalIlluminance(inoData);
    ss << c2 << " " << c3 << " " << val << "\n";
    sol=ss.str();
  }
  // g r <i>
  if(c2 == 114) {
    float val = 0;
    int c3=std::stoi(index);
    val = inoData[c3-1].GetReference();
    ss << c2 << " " << c3 << " " << val << "\n";
    sol=ss.str();
  } 
  if(c2 == 112) {
    // g p T
    if(index.at(0) == 84) {
      // POR AQUI CENAS
      float val=0;
      for(int i=0;i<N_inos;++i) {
		val+=inoData[i].GetInstantaneousPower();
      }
      ss << c2 << " " << "T" << " " << val << "\n";
      sol=ss.str();
    }
    // g p <i>
    else {
      float val = 0;
      int c3=std::stoi(index);
      val = inoData[c3-1].GetInstantaneousPower();
      ss << c2 << " " << c3 << " " << val << "\n";
      sol=ss.str();
    }
  }
  if(c2 == 101) {
    // g e T
    if(index.at(0) == 84) {
      float val=0;
      for(int i=0;i<N_inos;++i) {
		val+=inoData[i].GetAccumulatedEnergy();
      }
      ss << c2 << " " << "T" << " " << val << "\n";
      sol=ss.str();
    }
    // g e <i>
    else {
      float val = 0;
      int c3=std::stoi(index);
      val = inoData[c3-1].GetAccumulatedEnergy();
      ss << c2 << " " << c3 << " " << val << "\n";
      sol=ss.str();
    }
  } 
  if(c2 == 99) {
    // g c T
    if(index.at(0) == 84) {
      // POR CENAS AQUI
      float val=0;
      for(int i=0;i<N_inos;++i) {
		val+=inoData[i].GetComfortError();
      }
      ss << c2 << " " << "T" << " " << val << "\n";
      sol=ss.str();
    }
    // g c <i>
    else {
      float val = 0;
      int c3=std::stoi(index);
      val = inoData[c3-1].GetComfortError();
      ss << c2 << " " << c3 << " " << val << "\n";
      sol=ss.str();
    }
  } 
  if(c2 == 118) {
    // g v T
    if(index.at(0) == 84) {
      float val=0;
      for(int i=0;i<N_inos;++i) {
		val+=inoData[i].GetComfortVariance();
      }
      ss << c2 << " " << "T" << " " << val << "\n";
      sol=ss.str();
    }
    // g v <i>
    else {
      float val = 0;
      int c3=std::stoi(index);
      val = inoData[c3-1].GetComfortVariance();
      ss << c2 << " " << c3 << " " << val << "\n";
      sol=ss.str();
    }
  }
  ss.str(std::string()); 
  return sol;
}

std::string resolve_buffer(std::string comm){
  std::string sol1;
  std::string sol2;
  std::ostringstream ss;
  char c2 = comm.at(0);
  int space = comm.find_last_of(" ");
  std::string index = comm.substr(space+1);
  int c3 = std::stoi(index);
  bool variable = 0;
  
  if(c2==100) variable=0;
  else variable=1;

  sol2=inoData[c3-1].GetLastMinuteBuffer(variable);
  ss << "b" << " " << sol2;
  sol1=ss.str();
  return sol1;
}
