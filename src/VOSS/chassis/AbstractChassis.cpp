#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace voss::chassis {

AbstractChassis::AbstractChassis(controller_ptr default_controller) {
    this->default_controller = default_controller;
}

void AbstractChassis::move_task(controller_ptr controller, double max,
                                voss::Flags flags, double exitTime) {
//    int t = 0;
    pros::Task running_t([&, controller, flags, max]() {
        this->m.take();
        printf("mutex take\n");
        controller->reset();
        while (!this->execute(controller->get_command(flags & voss::Flags::REVERSE,
                                                      flags & voss::Flags::THRU),
                              max)) {
            printf("in loop\n");
            if (pros::competition::is_disabled()) {
                printf("exit bcuz of competition status\n");
                return;
            }

//            if (t > exitTime) {
//                printf("exit bcuz of time\n");
//                return;
//            }
//            t += 10;
            pros::delay(10);
        }

        printf("mutex give\n");
    });
    this->m.give();
    if (flags & voss::Flags::ASYNC) {
        pros::delay(10);
        printf("exit\n");
        return;
    }

    running_t.join();
}

void AbstractChassis::turn_task(controller_ptr controller, double max,
                                voss::Flags flags,
                                voss::AngularDirection direction,
                                double exitTime) {
    int t = 0;
    pros::Task running_t([&, controller, flags, direction, max]() {
        this->m.take();
        printf("mutex take\n");
        controller->reset();
        while (!this->execute(controller->get_angular_command(
                                 flags & voss::Flags::REVERSE,
                                 flags & voss::Flags::THRU, direction),
                             max)) {
            printf("in loop\n");
            if (pros::competition::is_disabled()) {
                return;
            }

//            if (t > exitTime) {
//                return;
//            }


            t += 10;
            pros::delay(10);
        }

        printf("mutex give\n");
    });
    this->m.give();
    if (flags & voss::Flags::ASYNC) {
        printf("exit\n");
        return;
    }
    running_t.join();
}

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
    printf("im here\n");
    this->move_task(controller, max, flags, exitTime);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           voss::AngularDirection direction, double exitTime) {
    this->turn(target, this->default_controller, max, flags, direction,
               exitTime);
}

void AbstractChassis::turn(double target, controller_ptr controller, double max,
                           voss::Flags flags, voss::AngularDirection direction,
                           double exitTime) {
    // this->m.take();

    controller->set_target({0, 0, 0}, false);
    controller->set_angular_target(target, flags & voss::Flags::RELATIVE);

    this->turn_task(controller, max, flags, direction, exitTime);
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
    // this->m.take();

    controller->set_target({target.x, target.y, 361},
                           flags & voss::Flags::RELATIVE);

    this->turn_task(controller, max, flags, direction, exitTime);
}

} // namespace voss::chassis