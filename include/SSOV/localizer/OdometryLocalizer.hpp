#pragma once

#include "SSOV/localizer/Localizer.hpp"

namespace ssov {
class OdometryLocalizer: public Localizer {
    private:
        Pose current_pose;
    public:
        OdometryLocalizer(uint32_t update_time = 10): Localizer(update_time) {};
        void update() override;
        Pose get_pose() override {
            std::lock_guard<pros::Mutex> guard(mtx);
            return current_pose;
        }
        void set_pose(Pose pose) override {
            std::lock_guard<pros::Mutex> guard(mtx);
            current_pose = pose;
        }
        virtual Pose get_local_change() = 0;
};
}