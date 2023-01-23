#include "pid.hpp"

double error;
double derivative;
double speed;

Pid::Pid(double _p, double _i = 0.0, double _d = 0.0) 
    : p(_p), i(_i), d(_d)
    {
    p = _p;
    i = _i;
    d = _d;
    }

Pid::apply(const double currentValue, const double targetValue){
    error = targetValue - currentValue;
    derivative = error - pe;
	if ((pe > 0 && error < 0) || (pe < 0 && error > 0))
		in = 0; // remove integral at zero error
	double speed = error * kp +in * ki + derivative * kd;

	// only let integral wind up if near the target
	if (fabs(error) < 15) {
		in += error;
	}

	pe = error;


    return speed;
}



Pid::PidOut(){
    return speed;
}