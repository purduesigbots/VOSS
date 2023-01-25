#pragma once

namespace legs {

class Pid
{
public:
    Pid(double _p, double _i = 0.0, double _d = 0.0) 
    : p(_p), i(_i), d(_d)
    {

    }
    
    double apply(const double currentValue, const double targetValue);
    
    /**
     * Automatically creates a task that runs the PID calculations on the 
     * passed references.
     * 
     * @param currentValue The variable holding the current state of the system
     * @param targetValue The variable we want the current state to reach
     * @param delay How many milleseconds for each iteration
     * 
     * @returns The amount of force to apply to reach the target value
    */
    double task(double& currentValue, double& targetValue, int delay = 10);

    double p, i, d;
};

} // namespace legs