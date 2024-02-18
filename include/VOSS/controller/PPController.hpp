#pragma once

#include "voss/controller/PIDController.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"
#include "voss/utils/flags.hpp"
#include "voss/utils/Point.hpp"

namespace voss::controller {
class PPController : public voss::controller::AbstractController {
  public:
    PPController(std::shared_ptr<voss::localizer::AbstractLocalizer> l);

    bool isArrived() const;
    chassis::DiffChassisCommand get_command(bool reverse, bool thru) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    void reset() override;

  protected:
    std::shared_ptr<PIDController> child = nullptr;
    double lookAheadDist;
  private:
    bool arrived = true;
    int closest();
    voss::Point absToLocal(voss::Point pt);
    voss::Point getLookAheadPoint(double lookAheadDist);

    template <typename T> static int sgn(T val) {
        return val >= 0 ? 1 : -1;
    }

    friend class PPControllerBuilder;
};
} // namespace voss::controller