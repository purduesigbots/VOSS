#pragma once

namespace voss::utils {

class PID {
  private:
    double kP, kI, kD;
    double prev_error, total_error;

  public:
    PID();
    PID(double kP, double kI, double kD);
    double update(double error);
    void reset();
    void set_constants(double kP, double kI, double kD);
};

} // namespace voss::utils