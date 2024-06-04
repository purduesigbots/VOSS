
#pragma once

namespace voss::utils {

class FeedForward {
  public:
    FeedForward() = default;
    FeedForward(double kS, double kV, double kA, double kD);
    double update(double velocity, double acceleration);
    void set_constants(double kS, double kV, double kA, double kD);

  private:
    double kS = 0 , kV = 0, kA = 0, kD = 0;

};

} // namespace voss
