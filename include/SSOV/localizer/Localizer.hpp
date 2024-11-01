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
            pros::Mutex mtx;
            std::unordered_set<pros::task_t> listeners;
        protected:
            Pose current_pose;
        public:
            Localizer(uint32_t update_time = 10): update_time(update_time) {};
            virtual void calibrate() = 0;
            virtual void update() = 0;
            void begin_localization();
            Pose get_pose() {
                std::lock_guard<pros::Mutex> guard(mtx);
                return current_pose;
            }
            void set_pose(Pose pose) {
                std::lock_guard<pros::Mutex> guard(mtx);
                current_pose = pose;
            }
            void add_listener(pros::task_t listener) {
                listeners.emplace(listener);
            }
            void remove_listener(pros::task_t listener) {
                listeners.erase(listener);
            }
    };
}