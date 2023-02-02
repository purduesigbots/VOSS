#include "legs/pid.hpp"
#include "api.h"
#include "math.h"

namespace legs {

Pid::Pid(double _p, double _i = 0.0, double _d = 0.0) 
    : p(_p), i(_i), d(_d)
    {
    p = _p;
    i = _i;
    d = _d;
    }

    double Pid::apply(const double currentValue, const double targetValue){
    error = targetValue - currentValue;
    derivative = error - pe;
	if ((pe > 0 && error < 0) || (pe < 0 && error > 0))
		in = 0; // remove integral at zero error
	speed = error * p + in * i + derivative * d;

	// only let integral wind up if near the target
	if (fabs(error) < 15) {
		in += error;
	}

	pe = error;

    return speed;
}

double Pid::pidOut(){
    return speed;
}

void Pid::task(double& currentValue, double& target, int delay){
    while(1){
        apply(currentValue, target);

        pros::delay(delay);
    }
}


}


