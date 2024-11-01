#include "SSOV/localizer/Localizer.hpp"

namespace ssov {
void Localizer::begin_localization() {
    calibrate();
    task = pros::Task::create([this]() {
        while (true) {
            mtx.take();
            update();
            mtx.give();
            for (const auto &listener : listeners) {
                pros::c::task_notify(listener);
            }
            pros::delay(update_time);
        }
    });
}
}