#include "defs.h"

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
		
	}
	// g O <i>
	if(c2 == 79) {
		
	}
	// g r <i>
	if(c2 == 114) {
		
	} 
	if(c2 == 79) {
		// g p T
		if(c3 == 84) {

		}
		// g p <i>
		else {

		}
	}
	if(c2 == 101) {
		// g e T
		if(c3 == 84) {

		}
		// g e <i>
		else {

		}
	} 
	if(c2 == 99) {
		// g c T
		if(c3 == 84) {

		}
		// g c <i>
		else {

		}
	} 
	if(c2 == 118) {
		// g v T
		if(c3 == 84) {

		}
		// g v <i>
		else {

		}
	} 
	return sol;
}