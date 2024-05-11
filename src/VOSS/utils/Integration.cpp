#include "VOSS/utils/Integration.hpp"

#include "VOSS/utils/Algorithms.hpp"

namespace voss::utils {

IntegralScan::IntegralScan(double a, double b, double eps, std::function<double(double)> f) {
    double m = (a + b) / 2;

    double fa = f(a);
    double fb = f(b);
    double fm = f(m);

    i = (b - a) / 8 * (
        fa + fm + fb
        + f(a + 0.9501 * (b - a))
        + f(a + 0.2311 * (b - a))
        + f(a + 0.6068 * (b - a))
        + f(a + 0.4860 * (b - a))
        + f(a + 0.8913 * (b - a))
    );
    if (i == 0.0) {
        i = b - a;
    }

    i *= eps / __DBL_EPSILON__;

    values.push_back(a);
    sums.push_back(0);

    helper(a, m, b, fa, fm, fb, f);
}

void IntegralScan::helper(double a, double m, double b, double fa, double fm, double fb, std::function<double(double)> f) {
    double h = (b - a) / 4;
    double ml = a + h;
    double mr = b - h;
    double fml = f(ml);
    double fmr = f(mr);

    double i1 = h / 1.5 * (fa + 4 * fm + fb);
    double i2 = h / 3 * (fa + 4 * (fml + fmr) + 2 * fm + fb);
    i1 = (16 * i2 - i1) / 15;

    if (i + (i1 - i2) == i || m <= a || m >= b) {
        values.push_back(b);
        sums.push_back(sums.back() + i1);
    } else {
        helper(a, ml, m, fa, fml, fm, f);
        helper(m, mr, b, fm, fmr, fb, f);
    }
}

double IntegralScan::end() {
    return this->sums.back();
}

double IntegralScan::lookup(double query) {
    if (query < values.front()) {
        return sums.front();
    }
    if (query > values.back()) {
        return sums.back();
    }

    int index = insert_index(values, query);

    if (values[index] == query) {
        return sums[index];
    }

    double sum_high = this->sums.at(index);
    double sum_low = this->sums.at(index - 1);
    double val_high = this->values.at(index);
    double val_low = this->values.at(index - 1);

    return sum_low + (query - val_low) * (sum_high - sum_low) / (val_high - val_low);
}

double IntegralScan::lookup_inverse(double query) {
    if (query < sums.front()) {
        return values.front();
    }
    if (query > sums.back()) {
        return values.back();
    }

    int index = insert_index(sums, query);

    if (sums[index] == query) {
        return values[index];
    }

    double sum_high = this->sums.at(index);
    double sum_low = this->sums.at(index - 1);
    double val_high = this->values.at(index);
    double val_low = this->values.at(index - 1);

    return val_low + (query - sum_low) * (val_high - val_low) / (sum_high - sum_low);
}

}