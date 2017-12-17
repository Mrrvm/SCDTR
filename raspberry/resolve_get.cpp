#include "resolve_get.h"

extern std::vector<Data> inoData;

std::string resolve_get(char c2, char c3) {

	std::string sol;
	std::ostringstream ss;
	// g l <i>
	if(c2 == 108) {
		float val = 0;
		//val = inoData[c3].GetIlluminance();
		ss << c2 << " " << c3 << val;
		sol=ss.str();
	}
	// g d <i>
	if(c2 == 100) {
		float val = 0; 
		// duty cycle in percentage!
		//val = inoData[c3].GetDutyCycle();
		ss << c2 << " " << c3 << val;
		sol=ss.str();
	}
	// g o <i>
	if(c2 == 111) {
		bool val;
		//val = inoData[c3].GetOccupancy();
		ss << c2 << " " << c3 << val;
		sol=ss.str();
	}
	// g L <i>
	if(c2 == 76) {
		float val = 0;
		//val = inoData[c3].GetIlluminanceLowerBound();
		ss << c2 << " " << c3 << val;
		sol=ss.str();
	}
	// g O <i>
	if(c2 == 79) {
		float val = 0;
		//val = inoData[c3].GetExternalIlluminance(std::vector<Data>);
		//sol << c2 << " " << c3 << " " << val;
		ss << c2 << " " << c3 << val;
		sol=ss.str();
	}
	// g r <i>
	if(c2 == 114) {
		float val = 0;
		//val = inoData[c3].GetReference();
		//sol << c2 << " " << c3 << " " << val;
		ss << c2 << " " << c3 << val;
		sol=ss.str();
	} 
	if(c2 == 79) {
		// g p T
		if(c3 == 84) {
			// POR AQUI CENAS
		}
		// g p <i>
		else {
			float val = 0;
			//val = inoData[c3].GetInstantaneousPower();
			//sol << c2 << " " << c3 << " " << val;
			ss << c2 << " " << c3 << val;
			sol=ss.str();
		}
	}
	if(c2 == 101) {
		// g e T
		if(c3 == 84) {
			float val = 0;
			//val = inoData[c3].GetAccumulatedEnergy();
			//sol << c2 << " " << c3 << " " << val;
			ss << c2 << " " << c3 << val;
			sol=ss.str();
		}
		// g e <i>
		else {
			float val = 0;
			//val = inoData[c3].GetInstantaneousPower();
			//sol << c2 << " " << c3 << " " << val;
			ss << c2 << " " << c3 << val;
			sol=ss.str();
		}
	} 
	if(c2 == 99) {
		// g c T
		if(c3 == 84) {
			// POR CENAS AQUI
		}
		// g c <i>
		else {
			float val = 0;
			//val = inoData[c3].GetComfortError();
			//sol << c2 << " " << c3 << " " << val;
		}
	} 
	if(c2 == 118) {
		// g v T
		if(c3 == 84) {
			// POR CENAS AQUI
		}
		// g v <i>
		else {
			float val = 0;
			//val = inoData[c3].GetComfortVariance();
			//sol << c2 << " " << c3 << " " << val;
		}
	} 
	return sol;
}