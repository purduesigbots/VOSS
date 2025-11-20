#pragma once

#include "SSOV/localizer/OdometryLocalizer.hpp"

#include <cmath>

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class HoloRobotOdom : public ssov::OdometryLocalizer {
    private:
        pros::MotorGroup front_left_motors;
        pros::MotorGroup front_right_motors;
        pros::MotorGroup back_left_motors;
        pros::MotorGroup back_right_motors;
        std::optional <pros::IMU> imu;
        double prev_front_left = 0;
        double prev_front_right = 0;
        double prev_back_left = 0;
        double prev_back_right = 0;
        double prev_angle = 0;
        double radial_velocity = 0;
        const double smoothing_factor = 0.3;
        const double motor_tpi = 45.46;
        const double rot_tpi = 4037.04908;
        const double tracking_wheel_offset = 4.716;
    public:
        explicit HoloRobotOdom(std::initializer_list<int8_t> front_left_ports,
                  std::initializer_list<int8_t> front_right_ports,
                  std::initializer_list<int8_t> back_left_ports,
                  std::initializer_list<int8_t> back_right_ports):
            ssov::OdometryLocalizer({}, 10),
            front_left_motors(front_left_ports),
            front_right_motors(front_right_ports),
            back_left_motors(back_left_ports),
            back_right_motors(back_right_ports){};
        HoloRobotOdom(std::initializer_list<int8_t> front_left_ports,
                  std::initializer_list<int8_t> front_right_ports,
                  std::initializer_list<int8_t> back_left_ports,
                  std::initializer_list<int8_t> back_right_ports,
                  uint8_t imu_port):
            ssov::OdometryLocalizer({}, 10),
            front_left_motors(front_left_ports),
            front_right_motors(front_right_ports),
            back_left_motors(back_left_ports),
            back_right_motors(back_right_ports),
            imu(imu_port){};

        void calibrate() {
            if (imu.has_value()){
                imu->reset();
                prev_angle = get_imu_angle();
            }
            front_left_motors.tare_position_all();
            front_right_motors.tare_position_all();
            back_left_motors.tare_position_all();
            back_right_motors.tare_position_all();
            prev_front_left = get_front_left_position();
            prev_front_right = get_front_right_position();
            prev_back_left = get_back_left_position();
            prev_back_right = get_back_right_position();
        }
        double get_front_left_position() const {
            double front_left_position = 0;
            for (const auto &pos : front_left_motors.get_position_all()) {
                front_left_position += pos / (motor_tpi * front_left_motors.size());
            }
            return front_left_position;
        }
        double get_front_left_velocity() const {
            double front_left_velocity = 0;
            for (const auto &vel : front_left_motors.get_actual_velocity_all()) {
                front_left_velocity += vel * 15 / (motor_tpi * front_left_motors.size());
            }
            return front_left_velocity;
        }
        double get_back_left_position() const {
            double back_left_position = 0;
            for (const auto &pos : back_left_motors.get_position_all()) {
                back_left_position += pos / (motor_tpi * back_left_motors.size());
            }
            return back_left_position;
        }
        double get_back_left_velocity() const {
            double back_left_velocity = 0;
            for (const auto &vel : back_left_motors.get_actual_velocity_all()) {
                back_left_velocity += vel * 15 / (motor_tpi * back_left_motors.size());
            }
            return back_left_velocity;
        }
        double get_front_right_position() const {
            double front_right_position = 0;
            for (const auto &pos : front_right_motors.get_position_all()) {
                front_right_position += pos / (motor_tpi * front_right_motors.size());
            }
            return front_right_position;
        }
        double get_front_right_velocity() const {
            double front_right_velocity = 0;
            for (const auto &vel : front_right_motors.get_actual_velocity_all()) {
                front_right_velocity += vel * 15 / (motor_tpi * front_right_motors.size());
            }
            return front_right_velocity;
        }
        double get_back_right_position() const {
            double back_right_position = 0;
            for (const auto &pos : back_right_motors.get_position_all()) {
                back_right_position += pos / (motor_tpi * back_right_motors.size());
            }
            return back_right_position;
        }
        double get_back_right_velocity() const {
            double back_right_velocity = 0;
            for (const auto &vel : back_right_motors.get_actual_velocity_all()) {
                back_right_velocity += vel * 15 / (motor_tpi * back_right_motors.size());
            }
            return back_right_velocity;
        }
        double get_imu_angle() const {
            return imu->get_rotation() * -1 * M_PI / 180;
        }

        double get_ang_vel() const {
            if(!imu.has_value()){
                double front_left_pos = get_front_left_position();
                double front_right_pos = get_front_right_position();
                double back_left_pos = get_back_left_position();
                double back_right_pos = get_back_right_position();
                double dtheta = (((front_right_pos + front_right_pos - front_left_pos - back_left_pos)/2)/(2 * -7.2)) - (((prev_front_right + prev_front_right - prev_front_left - prev_back_left)/2)/(2 * -7.2));
                return dtheta * 100;
            }
            else{
                return imu->get_gyro_rate().z * M_PI / 180;
            }
        }
            
        ssov::Pose get_local_change() override {
            double front_left_pos = get_front_left_position();
            double front_right_pos = get_front_right_position();
            double back_left_pos = get_back_left_position();
            double back_right_pos = get_back_right_position();
            double dtheta;
            if (imu.has_value()){
                double angle = get_imu_angle();
                dtheta = angle - prev_angle;
            }
            else{
                 dtheta = (((front_right_pos + front_right_pos - front_left_pos - back_left_pos)/2)/(2 * -7.2)) - (((prev_front_right + prev_front_right - prev_front_left - prev_back_left)/2)/(2 * -7.2));
            }
            double dx = sin(dtheta)*((front_left_pos - (((front_right_pos-back_left_pos)/-2) - ((back_left_pos-front_left_pos)/2)))-(prev_front_left - (((prev_front_right-prev_back_left)/-2) - ((prev_back_left-prev_front_left)/2))));
            double dy = cos(dtheta)*((front_right_pos+back_left_pos-front_left_pos-back_right_pos)/-4)-((prev_front_right+prev_back_left-prev_front_left-prev_back_right)/-4);
            radial_velocity = dy * 100;
            prev_front_left = front_left_pos;
            prev_front_right = front_right_pos;
            prev_back_left = back_left_pos;
            prev_back_right = back_right_pos;
            prev_angle += dtheta;
            return {dx, dy, dtheta};
        }

        ssov::Pose get_velocities() override {
            double front_left_velocity = get_front_left_velocity();
            double front_right_velocity = get_front_right_velocity();
            double back_left_velocity = get_back_left_velocity();
            double back_right_velocity = get_back_right_velocity();
            double ang_velocity = get_ang_vel();
            double tangential_velocity = front_left_velocity - ((front_right_velocity-back_left_velocity)/-2) - ((back_left_velocity-front_left_velocity)/2);
            double radial_velocity = (front_right_velocity-back_left_velocity)/-2;
            return {tangential_velocity, radial_velocity, ang_velocity};
        }
};