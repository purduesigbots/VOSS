#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace voss::chassis {

AbstractChassis::AbstractChassis(controller_ptr default_controller) {
    this->default_controller = default_controller;
}

void AbstractChassis::move_task(controller_ptr controller, double max,
                                voss::Flags flags, double exitTime) {
    int t = 0;
    pros::Task running_t([&, controller]() {
        controller->reset();
		//Loops until movement is complete or robot is disabled
        while (
            !this->execute(controller->get_command(flags & voss::Flags::REVERSE,
                                                   flags & voss::Flags::THRU),
                           max)) {
            if (pros::competition::is_disabled()) {
                return;
            }
            if (t > exitTime) {
                return;
            }

            t += 10;
            pros::delay(10);
        }
        // this->m.give();
    });

	//Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    running_t.join();

    // this->m.take();
    // this->m.give();
}

void AbstractChassis::turn_task(controller_ptr controller, double max,
                                voss::Flags flags, double exitTime) {
    int t = 0;
    pros::Task running_t([&, controller]() {
        controller->reset();
		//Loops until movement is complete or robot is disabled
        while (!this->execute(
            controller->get_angular_command(flags & voss::Flags::REVERSE,
                                            flags & voss::Flags::THRU),
            max)) {
            if (pros::competition::is_disabled()) {
                return;
            }

            if (t > exitTime) {
                return;
            }

            t += 10;
            pros::delay(10);
        }
    });

	//Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    running_t.join();
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
    this->move(pose_target, controller, max, flags, exitTime);
}

void AbstractChassis::move(Pose target, controller_ptr controller, double max,
                           voss::Flags flags, double exitTime) {
    // this->m.take();

    controller->set_target(target, flags & voss::Flags::RELATIVE);

    this->move_task(controller, max, flags, exitTime);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           double exitTime) {
    this->turn(target, this->default_controller, max, flags, exitTime);
}

void AbstractChassis::turn(double target, controller_ptr controller, double max,
                           voss::Flags flags, double exitTime) {
    // this->m.take();

    controller->set_target({0, 0, 0}, false);
    controller->set_angular_target(target, flags & voss::Flags::RELATIVE);

    this->turn_task(controller, max, flags, exitTime);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags,
                              double exitTime) {
    this->turn_to(target, this->default_controller, max, flags, exitTime);
}

void AbstractChassis::turn_to(Point target, controller_ptr controller,
                              double max, voss::Flags flags, double exitTime) {
    // this->m.take();

    controller->set_target({target.x, target.y, 361},
                           flags & voss::Flags::RELATIVE);

    this->turn_task(controller, max, flags, exitTime);
}

} // namespace voss::chassis