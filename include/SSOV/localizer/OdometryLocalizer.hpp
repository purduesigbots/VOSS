#pragma once

#include "Localizer.hpp"


namespace ssov {
class OdometryLocalizer: public Localizer {
    private:
        Pose current_pose;
        Pose local_offset;
        Pose adjusted_pose;
    public:
        OdometryLocalizer(Pose local_offset = {}, uint32_t update_time = 10): Localizer(update_time), local_offset(local_offset) {};
        void update() override;
        Pose get_pose() override {
            std::lock_guard<pros::Mutex> guard(mtx);
            adjusted_pose = current_pose - local_offset;
            adjusted_pose.theta = to_degrees(adjusted_pose.theta); 
            return adjusted_pose;
        }
        void set_pose(Pose pose) override {
            std::lock_guard<pros::Mutex> guard(mtx);
            pose.theta = to_radians(pose.theta);
            current_pose = pose + local_offset;
        }
        virtual Pose get_local_change() = 0;
};
}