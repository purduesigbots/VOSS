#pragma once

#include "SSOV/localizer/AbstractTrackingWheel.hpp"
#include <memory>
#include "pros/adi.hpp"

namespace ssov {

class ADITrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::adi::Encoder> encoder;

  protected:
    double get_raw_position() override {
        return encoder->get_value();
    }

  public:
    ADITrackingWheel(int adi_port, double tpi):
    encoder(std::make_unique<pros::adi::Encoder>(abs(adi_port), abs(adi_port) + 1, adi_port < 0)) {
      set_tpi(tpi);
    };

    ADITrackingWheel(int smart_port, int adi_port): encoder(std::make_unique<pros::adi::Encoder>(
        std::make_tuple(smart_port, abs(adi_port), abs(adi_port) + 1), adi_port < 0
    )) {};

    void reset() override {
        encoder->reset();
    }
};

}