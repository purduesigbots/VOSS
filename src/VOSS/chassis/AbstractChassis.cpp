#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace voss::chassis {

AbstractChassis::AbstractChassis(controller_ptr default_controller) {
    this->default_controller = std::move(default_controller);
}

void AbstractChassis::move_task(controller_ptr controller, double max,
                                voss::Flags flags, double exitTime) {

    this->task =
        std::make_unique<pros::Task>([&, controller, flags, max, exitTime]() {
            controller->reset();
            while (!this->execute(
                controller->get_command(flags & voss::Flags::REVERSE,
                                        flags & voss::Flags::THRU),
                max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(10);
            }
            this->task_running = false;
        });

	//Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void AbstractChassis::turn_task(controller_ptr controller, double max,
                                voss::Flags flags,
                                voss::AngularDirection direction,
                                double exitTime) {

    this->task = std::make_unique<pros::Task>(
        [&, controller, flags, direction, max, exitTime]() {
            controller->reset();
            while (!this->execute(controller->get_angular_command(
                                      flags & voss::Flags::REVERSE,
                                      flags & voss::Flags::THRU, direction),
                                  max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(10);
            }
            this->task_running = false;
        });

	//Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }
    this->task->join();
}


//Overloaded constructors move functions to allow for different parameters
void AbstractChassis::move(Point target, double max, voss::Flags flags,
                           double exitTime) {
    this->move(target, this->default_controller, max, flags, exitTime);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags,
                           double exitTime) {
    this->move(target, this->default_controller, max, flags, exitTime);
}

void AbstractChassis::move(Point target, controller_ptr controller, double max,
                           voss::Flags flags, double exitTime) {
    Pose pose_target = Pose{target.x, target.y, 361};
    this->move(pose_target, std::move(controller), max, flags, exitTime);
}

void AbstractChassis::move(Pose target, controller_ptr controller, double max,
                           voss::Flags flags, double exitTime) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    controller->set_target(target, flags & voss::Flags::RELATIVE);

    this->move_task(std::move(controller), max, flags, exitTime);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           voss::AngularDirection direction, double exitTime) {
    this->turn(target, this->default_controller, max, flags, direction,
               exitTime);
}

void AbstractChassis::turn(double target, controller_ptr controller, double max,
                           voss::Flags flags, voss::AngularDirection direction,
                           double exitTime) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    controller->set_target({0, 0, 0}, false);
    controller->set_angular_target(target, flags & voss::Flags::RELATIVE);

    this->turn_task(std::move(controller), max, flags, direction, exitTime);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags,
                              voss::AngularDirection direction,
                              double exitTime) {
    this->turn_to(target, this->default_controller, max, flags, direction,
                  exitTime);
}

void AbstractChassis::turn_to(Point target, controller_ptr controller,
                              double max, voss::Flags flags,
                              voss::AngularDirection direction,
                              double exitTime) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    controller->set_target({target.x, target.y, 361},
                           flags & voss::Flags::RELATIVE);

    this->turn_task(std::move(controller), max, flags, direction, exitTime);
}

} // namespace voss::chassis