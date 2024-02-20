//
// Created by Rocky Chen on 2024/1/18.
//

#include "VOSS/controller/PPController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {
PPController::PPController(
    std::shared_ptr<voss::localizer::AbstractLocalizer> l)
    : AbstractController(l) {
    this->arrived = false;
}

chassis::DiffChassisCommand PPController::get_command(bool reverse, bool thru) {
    this->counter += 10;
    voss::Point currentPose = this->l->get_position();
    double heading = this->l->get_orientation_rad();
    double overallLinearError =
        voss::Point::getDistance(currentPose, this->target_path.back());
    int minIndex = this->closest();
    bool chainedExecutable = false;
    printf("im here\n");

    int dir = reverse ? -1 : 1;
    printf("%b", !this->arrived && minIndex < this->target_path.size() - 1);
    if (!this->arrived && minIndex < this->target_path.size() - 1) {

        double toNextPointError = voss::Point::getDistance(
            currentPose, this->target_path.at(minIndex));
        voss::Point lookAheadPt =
            absToLocal({currentPose.x, currentPose.y, heading},
                       (this->getLookAheadPoint(std::max(
                           std::min(lookAheadDist, overallLinearError), 5.0))));

        double linearError = lookAheadPt.y / this->lookAheadDist;
        double angularError = lookAheadPt.x / this->lookAheadDist;
        printf("look ahead pt: x: %.2f, y: %.2f\n", lookAheadPt.x, lookAheadPt.y);

        double angularOut = this->angular_pid(angularError);
        double linearOut;
        double m =
            fabs((currentPose.y - target.y) / (currentPose.x - target.x));

        if (thru &&
            currentPose.y + this->min_error >=
                (-1.0 / m) * (currentPose.x + min_error -
                              this->target_path.back().x) +
                    this->target_path.back().y &&
            minIndex >= this->target_path.size() - 2) {
            chainedExecutable = true;
        }

        if (total_lin_err <= exit_error) {
            if (thru) {
                chainedExecutable = true;
            } else {
                total_lin_err = 0;
                close += 10;
            }
        } else {
            close = 0;
        }

        if (close > settle_time) {
            return chassis::DiffChassisCommand{chassis::Stop{}};
        }

        if (this->counter > 400) {
            if ((fabs(overallLinearError - this->prevOverallLinearError) <
                     0.1 &&
                 fabs(angularError - this->prev_ang_err) <
                     voss::to_radians(0.1)) ||
                (toNextPointError - this->prevToNextPointError > 3 &&
                 this->prevIndex == minIndex)) {
                this->close_2 += 10;
            } else {
                this->close_2 = 0;
            }
        }

        if (this->close_2 > this->settle_time * 2) {
            this->arrived = true;
            return chassis::DiffChassisCommand{chassis::Stop{}};
        }

        linearOut = linear_pid(linearError);
        if (minIndex != this->target_path.size() - 1 && thru) {
            linearOut =
                copysign(fmax(fabs(linearOut), this->min_speed), linearOut);
        }
        linearOut *= dir;

        linearOut *= cos(angularError);

        this->prevOverallLinearError = overallLinearError;
        this->prevToNextPointError = toNextPointError;
        this->prevIndex = minIndex;

        if (chainedExecutable) {
            return chassis::DiffChassisCommand{chassis::diff_commands::Chained{
                linearOut + angularOut, linearOut - angularOut}};
        }

        return chassis::DiffChassisCommand{chassis::diff_commands::Voltages{
            linearOut - angularOut, linearOut + angularOut}};
    }

    this->arrived = true;
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

chassis::DiffChassisCommand
PPController::get_angular_command(bool reverse, bool thru,
                                  voss::AngularDirection direction) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
};

bool voss::controller::PPController::isArrived() const {
    return this->arrived;
}

int PPController::closest() {
    auto path = this->target_path;
    voss::Point robot = this->l->get_position();
    double dx = path[0].x - robot.x;
    double dy = path[0].y - robot.y;
    double minD = sqrt(dx * dx + dy * dy);
    int minIndex = 0;
    for (int i = 1; i < path.size(); i++) {
        dx = path[i].x - robot.x;
        dy = path[i].y - robot.y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < minD) {
            minD = dist;
            minIndex = i;
        }
    }
    return minIndex;
}

voss::Point PPController::absToLocal(voss::Pose currentPose, voss::Point pt) {
    double dx = pt.x - currentPose.x;
    double dy = pt.y - currentPose.y;

    // apply rotation matrix
    double x = dy * cos(currentPose.theta) - dx * sin(currentPose.theta);
    double y = dy * sin(currentPose.theta) + dx * cos(currentPose.theta);

    return {x, y};
}

voss::Point PPController::getLookAheadPoint(double lookAheadDist) {
    voss::Point robot = this->l->get_position();
    auto path = this->target_path;

    double dx = path[path.size() - 1].x - robot.x;
    double dy = path[path.size() - 1].y - robot.y;
    double dToEnd = sqrt(dx * dx + dy * dy);
    if (dToEnd < lookAheadDist) {
        return path.at(path.size() - 1);
    }

    voss::Point lookAheadPt(0, 0);

    for (int i = 1; i < path.size(); i++) {
        voss::Point coord = path[i];
        voss::Point prevCoord = path[i - 1];

        // if suitable distance is found
        if (voss::Point::getDistance(coord, robot) > lookAheadDist &&
            voss::Point::getDistance(prevCoord, robot) < lookAheadDist &&
            this->closest() < i) {

            // interpolation
            double prevX = prevCoord.x;
            double prevY = prevCoord.y;

            double currX = coord.x;
            double currY = coord.y;

            double minT = 0;
            double maxT = 1;

            double newX = prevX;
            double newY = prevY;

            int iterations = 10;

            // binary approximation
            for (int z = 0; z < iterations; z++) {
                double midT = (minT + maxT) / 2.0;
                newX = prevX * (1 - midT) + currX * midT;
                newY = prevY * (1 - midT) + currY * midT;

                lookAheadPt = {newX, newY};

                double distToSelf =
                    voss::Point::getDistance(lookAheadPt, robot);

                if (distToSelf < lookAheadDist) {
                    minT = midT;
                } else {
                    maxT = midT;
                }
            }
            return lookAheadPt;
        }
    }

    lookAheadPt = path.at(this->closest());
    return lookAheadPt;
}

double PPController::linear_pid(double error) {
    total_lin_err += error;

    double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
                   linear_kI * total_lin_err;

    this->prev_lin_err = error;

    return speed;
}

double PPController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

void PPController::reset() {
    this->prev_lin_err = 0;
    this->total_lin_err = 0;
    this->prev_ang_err = 0;
    this->total_ang_err = 0;
    this->can_reverse = false;
    this->counter = 0;
    this->turn_overshoot = false;
    this->arrived = false;
}

} // namespace voss::controller