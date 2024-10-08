#include "PPController2.hpp"
#include "../../../include/VOSS/controller/PPController2.hpp"
#include "VOSS/utils/angle.hpp"




namespace voss::controller {

PPController2::PPController2(PPController2::Params params):
    linear_pid(params.lin_kp, params.lin_ki, params.lin_kd),
    angular_pid(params.ang_kp, params.ang_ki, params.ang_kd),
    look_ahead_dist(params.look_ahead_dist), track_width(params.track_width) 
{

}

std::shared_ptr<PPController2>
PPController2::create_controller(PPController2::Params params) 
{
    return std::move(std::make_shared<PPController2>(params));
}

chassis::DiffChassisCommand
PPController2::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                          std::shared_ptr<AbstractExitCondition> ec,
                          const velocity_pair& v_pair, bool reverse,
                          bool thru) 
{
    //basic drive setup
    double left_velo, right_velo;
    Point current_pos = l->get_position();
    int dir = reverse ? -1 : 1;
    
    //get the closest target point to the robot
    int index  = this->get_closest(l->get_pose());

    Point lookahead_point;

    //if no lookahead point was found, make the lookahead point the closet point
    if(this->get_lookahead_pt(l->get_pose(), index) == std::nullopt)
    {
        lookahead_point = Point{this->target_path.at(index).x, this->target_path.at(index).y};
    }else {
        lookahead_point = Point{this->get_lookahead_pt(l->get_pose(), index)->x,
         this->get_lookahead_pt(l->get_pose(), index)->y};
    }

    //deal with when the robot is close to the end
    if (Point::getDistance(lookahead_point, {this->target_path.back().x,
                                           this->target_path.back().y}) <
            this->look_ahead_dist &&
        index == target_path.size() - 1) 
    {
        const Pose target = target_path.back();

        auto vel_pair =
            pid_controller_for_ending(l->get_pose(), target, reverse);
        left_velo = vel_pair.first;
        right_velo = vel_pair.second;
    }else {
        // implement pid to get to point, dont care about heading
        auto error = get_relative_error(l->get_pose(), lookahead_point);
        
        double linear_velo  = linear_pid.update(error.x / this->look_ahead_dist * dir);
        double angular_velo = angular_pid.update(error.y / this->look_ahead_dist);

        left_velo = dir * (linear_velo -  angular_velo);
        right_velo = dir * (linear_velo + angular_velo);

        double vMax = std::max(fabs(left_velo), fabs(right_velo));
        if(vMax > 100)
        {
            left_velo *= 100 / vMax;
            right_velo *= 100 / vMax;
        }
    }

    //if the robot is not within a certian distance, move
    if (!ec->is_met(l->get_pose(),
                    thru)) { // we might not want thru for pp but who knows
        return chassis::diff_commands::Voltages{left_velo, right_velo};
    }
    //if the robot is at the last point in the target, stop the robot
    return chassis::Stop();
}

std::pair<double, double>
PPController2::pid_controller_for_ending(const Pose& current_pos,
                                        const Pose& target, bool reverse) 
{
    int dir = reverse ? -1 : 1;
    double current_angle = current_pos.theta.value();
    bool noPose = !target.theta.has_value();

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);

    double angle_error = atan2(dy, dx) - current_angle;

    if (reverse) {
        angle_error = atan2(-dy, -dx) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed = linear_pid.update(distance_error) * dir;

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;


        if (noPose) {
            ang_speed = 0; // disable turning when close to the point to prevent
                           // spinning
        } else {
            // turn to face the finale pose angle if executing a pose movement
            double poseError = target.theta.value() - current_angle;

            while (fabs(poseError) > M_PI)
                poseError -= 2 * M_PI * poseError / fabs(poseError);
            ang_speed = angular_pid.update(poseError);
        }

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else if (distance_error < 2 * min_error) {
        // scale angular speed down to 0 as distance_error approaches min_error
        ang_speed = angular_pid.update(angle_error);
        ang_speed *= (distance_error - min_error) / min_error;
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }
    lin_speed = std::max(-100.0, std::min(100.0, lin_speed));
    return {lin_speed - ang_speed, lin_speed + ang_speed};
}

int PPController2::get_closest(const Pose& current_pose)
{
    // calculate the distance from the robot to the end of the path
    double dx = target_path.front().x - current_pose.x;
    double dy = target_path.front().y - current_pose.y;
    double min_dist = sqrt(dx * dx + dy * dy);
    int min_index = 0;
    //iterate over all target path points to find the closest point to the robot
    for(int i  = 0; i < target_path.size(); i++)
    {
        dx = this->target_path.at(i).x - current_pose.x;
        dy = this->target_path.at(i).y - current_pose.y;
        double dist = sqrt(dx * dx + dy * dy);
        if(dist < min_dist)
        {
            min_index = i;
            min_dist = dist;
        }
    }
    // return the index within target_path of the closest point
    return min_index;
}

std::optional<Point> PPController2::get_lookahead_pt(const Pose& robot, int index)
{
    //start and end points of line segment
    Pose current_point = target_path.at(index + 1);
    Pose next_point = target_path.at(index);

    return circle_line_intersect(robot, current_point, next_point);
}  

std::optional<Point> PPController2::circle_line_intersect(const Pose& robot, 
    const Pose& start_pose, const Pose& end_pose) const
{
    // convert pose to points
    Point start_point = {start_pose.x, start_pose.y};
    Point end_point = {end_pose.x, end_pose.y};
    // vector that represents the line segment 
    Point d = end_point - start_point;
    // vector of the start point with respect to the origin
    Point f = start_point - Point{robot.x, robot.y};
    // length of line segment
    double a = d * d;
    // no idea wtf this is, copied from dawgma paper
    double b = 2 * (f * d);
    // again no idea, copied from paper
    double c = f * f - this->look_ahead_dist * this->look_ahead_dist;
    // discriminant of some quadratic equation
    double discriminant = b * b - 4 * a * c;
    
    if(discriminant < 0)
    {
        // theres no intersent, so return null
        return std::nullopt; 
    }else{
        // some quadratic equation from dawgma
        discriminant = sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        if( t1 >= 0 && t2 <= 1)
        {
            return start_point + Point{t1 * d.x, t1 * d.y};
        }else if (t2 >= 0 && t1 <= 1){   
            return start_point + Point{t2 * d.x, t2 + d.y};
        }else{
            return std::nullopt;
        }
    }

}

static Point get_relative_error(const Pose& robot, const Point& point)
{
    //x and y coordinates of the position vector of the point with respect to the robot position
    double dx = point.x - robot.x;
    double dy = point.y - robot.y;

    //robot heading
    double h =  robot.theta.value();

    //error between the robot and the point with respect to the robot heading
    float r_x = dy * cos(h) - dx * sin(h);
    float r_y = dy * sin(h) + dx * cos(h);

    return Point{r_x, r_y};

    
}

void PPController2::reset()
{
    linear_pid.reset();
    angular_pid.reset();
}

}