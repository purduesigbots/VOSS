#pragma once

#include "SSOV/localizer/Localizer.hpp"

namespace ssov {
class OdometryLocalizer: public Localizer {
    public:
        OdometryLocalizer(uint32_t update_time = 10): Localizer(update_time) {};
        void update() override;
        virtual Pose get_local_change() = 0;
};
}