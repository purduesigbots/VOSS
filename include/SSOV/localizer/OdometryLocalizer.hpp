#pragma once

#include "SSOV/localizer/Localizer.hpp"

namespace ssov {
class OdometryLocalizer: public Localizer {
    private:
        Pose current_pose;
        Pose local_offset;
    public:
        OdometryLocalizer(Pose local_offset = {}, uint32_t update_time = 10): Localizer(update_time), local_offset(local_offset) {};
        void update() override;
        Pose get_pose() override {
            std::lock_guard<pros::Mutex> guard(mtx);
            return current_pose - local_offset;
        }
        void set_pose(Pose pose) override {
            std::lock_guard<pros::Mutex> guard(mtx);
            current_pose = pose + local_offset;
        }
        virtual Pose get_local_change() = 0;
};
}