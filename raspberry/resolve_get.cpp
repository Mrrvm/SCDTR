#include "resolve_get.h"

std::string resolve_get(char c2, char c3) {

	std::string sol;
	// g l <i>
	if(c2 == 108) {
		float val = 0;
		//val = arduino[c3].GetIlluminance();
		sol << c2 << " " << c3 << " " << val;
	}
	// g d <i>
	if(c2 == 100) {
		float val = 0; 
		// duty cycle in percentage!
		//val = arduino[c3].GetDutyCycle();
		sol << c2 << " " << c3 << " " << val;
	}
	// g o <i>
	if(c2 == 100) {
		bool val;
		//val = arduino[c3].GetOccupancy();
		sol << c2 << " " << c3 << " " << val;
	}
	// g L <i>
	if(c2 == 76) {
		float val = 0;
		//val = arduino[c3].GetIlluminanceLowerBound();
		sol << c2 << " " << c3 << " " << val;
	}
	// g O <i>
	if(c2 == 79) {
		float val = 0;
		//val = arduino[c3].GetExternalIlluminance(std::vector<Data>);
		sol << c2 << " " << c3 << " " << val;
	}
	// g r <i>
	if(c2 == 114) {
		float val = 0;
		//val = arduino[c3].GetReference();
		sol << c2 << " " << c3 << " " << val;
	} 
	if(c2 == 79) {
		// g p T
		if(c3 == 84) {
			// POR AQUI CENAS
		}
		// g p <i>
		else {
			float val = 0;
			//val = arduino[c3].GetInstantaneousPower();
			sol << c2 << " " << c3 << " " << val;
		}
	}
	if(c2 == 101) {
		// g e T
		if(c3 == 84) {
			float val = 0;
			//val = arduino[c3].GetAccumulatedEnergy();
			sol << c2 << " " << c3 << " " << val;
		}
		// g e <i>
		else {
			float val = 0;
			//val = arduino[c3].GetInstantaneousPower();
			sol << c2 << " " << c3 << " " << val;
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
			//val = arduino[c3].GetComfortError();
			sol << c2 << " " << c3 << " " << val;
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
			//val = arduino[c3].GetComfortVariance();
			sol << c2 << " " << c3 << " " << val;
		}
	} 
	return sol;
}