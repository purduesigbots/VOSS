#pragma once

#include <mutex>
#include <unordered_set>

#include "SSOV/common/Pose.hpp"
#include "pros/rtos.hpp"

namespace ssov {
    class Localizer {
        private:
            pros::task_t task;
            uint32_t update_time;
            std::unordered_set<pros::task_t> listeners;
        protected:
            pros::Mutex mtx;
        public:
            int imu_dir = 1;
            Localizer(uint32_t update_time = 10): update_time(update_time) {};
            virtual void calibrate() = 0;
            virtual void update() = 0;
            void begin_localization();
            virtual Pose get_velocities() = 0;
            virtual Pose get_pose() = 0;
            virtual void set_pose(Pose pose) = 0;
            void add_listener(pros::task_t listener) {
                listeners.emplace(listener);
            }
            void remove_listener(pros::task_t listener) {
                listeners.erase(listener);
            }
    };
}