#pragma once

namespace legs {

class Pid
{
public:
    Pid(double p, double i, double d);
    
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
    void task(double& currentValue, double& targetValue, int delay = 10);

    double pidOut();

    double p, i, d;
    
private:
    double in, pe, speed, error, derivative;
};

} // namespace legs