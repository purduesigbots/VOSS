#pragma once

#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include "pros/motors.hpp"
#include <memory>

namespace voss::localizer {

class IMETrackingWheel : public AbstractTrackingWheel {
private:
    std::unique_ptr<pros::v5::Motor> encoder;
protected:
    double get_raw_position() override;
public:
    IMETrackingWheel(int port);
    void reset() override;
};

}