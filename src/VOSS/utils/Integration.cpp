#include "VOSS/utils/Integration.hpp"

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

    values.push_back(0);
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
    int start = 0;
    int end = values.size() - 1;

    while (start <= end) {
        int mid = (start + end) / 2;
        double comp = this->values.at(mid);
        if (query == comp) {
            return this->sums.at(mid);
        } else if (query < comp) {
            end = mid - 1;
        } else {
            end = mid + 1;
        }
    }

    double sum_high = this->sums.at(end + 1);
    double sum_low = this->sums.at(end);
    double val_high = this->values.at(end + 1);
    double val_low = this->values.at(end);

    return sum_low + (query - val_low) * (sum_high - sum_low) / (val_high - val_low);
}

double IntegralScan::lookup_inverse(double query) {
    int start = 0;
    int end = sums.size() - 1;

    while (start <= end) {
        int mid = (start + end) / 2;
        double comp = this->sums.at(mid);
        if (query == comp) {
            return this->values.at(mid);
        } else if (query < comp) {
            end = mid - 1;
        } else {
            end = mid + 1;
        }
    }

    double sum_high = this->sums.at(end + 1);
    double sum_low = this->sums.at(end);
    double val_high = this->values.at(end + 1);
    double val_low = this->values.at(end);

    return val_low + (query - sum_low) * (val_high - val_low) / (sum_high - sum_low);
}

}