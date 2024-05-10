#pragma once

#include <functional>
#include <vector>

namespace voss::utils {

class IntegralScan {
  private:
    std::vector<double> values;
    std::vector<double> sums;
    double i;

    void helper(double a, double m, double b, double fa, double fm, double fb, std::function<double(double)> f);

  public:
    IntegralScan(double a, double b, double eps, std::function<double(double)> f);

    double end();

    double lookup(double query);

    double lookup_inverse(double query);
};

}