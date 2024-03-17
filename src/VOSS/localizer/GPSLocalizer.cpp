#include "VOSS/localizer/GPSLocalizer.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::localizer {
GPSLocalizer::GPSLocalizer(int gps_port)
    : gps_port(gps_port), gps(std::make_unique<pros::GPS>(gps_port)) {
}

void GPSLocalizer::update() {
    double new_x, new_y, new_theta;
    auto [gps_x, gps_y] = this->gps->get_position();
    gps_x = mm_to_inch(gps_x);
    gps_y = mm_to_inch(gps_y);
    double gps_theta = voss::to_radians(this->gps->get_heading());

    new_theta = gps_theta + this->theta_offset;
    new_x = (gps_x + this->x_offset) * cos(this->theta_offset) - (gps_y + this->y_offset) * sin(this->theta_offset);
    new_y = (gps_x + this->x_offset) * sin(this->theta_offset) + (gps_y + this->y_offset) * cos(this->theta_offset);


    this->pose = voss::AtomicPose{new_x, new_y, new_theta};
}

void GPSLocalizer::calibrate() {
}

void GPSLocalizer::set_pose(Pose pose) {
    double delta_x, delta_y, delta_theta;
    delta_x = pose.x - mm_to_inch(this->gps->get_position().x);//in
    delta_y = pose.y - mm_to_inch(this->gps->get_position().y);//in
    delta_theta = pose.theta.has_value() ? voss::to_radians(pose.theta.value() - this->gps->get_heading()) : 0;

    this->AbstractLocalizer::set_pose(pose);

    this->theta_offset = delta_theta;
    this->x_offset = delta_x;
    this->y_offset = delta_y;
}


constexpr double GPSLocalizer::mm_to_inch(double cm) {
    return cm * 39.3701;
}

}; // namespace voss::localizer
