//
// Created by Rocky Chen on 2024/1/18.
//

#include "VOSS/controller/PPController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"

namespace voss::controller {
PPController::PPController(std::shared_ptr<voss::localizer::AbstractLocalizer> l): AbstractController(l) {
    this->pidController->reset();
}

bool voss::controller::PPController::isArrived() const {
    return this->arrived;
}

int PPController::closest(const std::vector<voss::Point> &path) {
    voss::Point robot = this->localizer->get_position();
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

voss::Point PPController::absToLocal(voss::Point pt) {
    voss::Pose currentPos = this->localizer->get_pose();
    double dx = pt.x - currentPos.x;
    double dy = pt.y - currentPos.y;

    // apply rotation matrix
    double x = dy * cos(currentPos.theta) - dx * sin(currentPos.theta);
    double y = dy * sin(currentPos.theta) + dx * cos(currentPos.theta);

    return {x, y};
}

voss::Point PPController::getLookAheadPoint(const std::vector<voss::Point> &path, double lookAheadDist) {
    voss::Point robot = this->localizer->get_position();

    double dx = path[path.size() - 1].x - robot.x;
    double dy = path[path.size() - 1].y - robot.y;
    double dToEnd = sqrt(dx * dx + dy * dy);
    if (dToEnd < lookAheadDist) {
        return path[path.size() - 1];
    }

    voss::Point lookAheadPt(0, 0);

    for (int i = 1; i < path.size(); i++) {
        voss::Point coord = path[i];
        voss::Point prevCoord = path[i - 1];

        // if suitable distance is found
        if (voss::Point::getDistance(coord, robot) > lookAheadDist &&
            voss::Point::getDistance(prevCoord, robot) < lookAheadDist &&
            this->closest(path) < i) {

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

                double distToSelf = voss::Point::getDistance(lookAheadPt, robot);

                if (distToSelf < lookAheadDist) {
                    minT = midT;
                } else {
                    maxT = midT;
                }
            }
            return lookAheadPt;
        }
    }

    lookAheadPt = path[this->closest(path)];
    return lookAheadPt;
}

void PPController::reset() {
    this->pidController->reset();
    this->arrived = false;
}

chassis::DiffChassisCommand
PPController::get_command(bool reverse, bool thru) {
    voss::Point p = this->localizer->get_position();
    double overallLinearError = voss::Point::getDistance(p, *this->target_path.end());
    int minIndex = this->closest(this->target_path);


    int dir = reverse ? -1 : 1;
    if (!this->arrived && minIndex < this->target_path.size() - 1) {
        double toNextPointError = voss::Point::getDistance(p, this->target_path.at(minIndex));
        voss::Point lookAheadPt = this->absToLocal(
                (this->getLookAheadPoint(this->target_path, std::max(std::min(lookAheadDist, overallLinearError), 5.0))));
        voss::Pose lookAheadPose = {lookAheadPt.x, lookAheadPt.y, 361}; // 361 is a flag to indicate that the theta is not set
        this->pidController->set_target(lookAheadPose, false);
                }

    return this->pidController->get_command(reverse, thru);
}
chassis::DiffChassisCommand PPController::get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
                        };
} // namespace voss::controller