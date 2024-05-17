
#pragma once
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/FeedFwd.hpp"
#include "VOSS/utils/PID.hpp"
namespace voss::controller {
class PPController : virtual public IsPathFollowController,
                     private std::enable_shared_from_this<PPController> {
  public:
    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse,
                bool thru, std::shared_ptr<AbstractExitCondition> ec) override;

    chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                        bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset();

    std::shared_ptr<PPController> get_ptr();
  private:
    utils::FeedForward left_ffwd;
    utils::FeedForward right_ffwd;
    utils::PID left_pid;
    utils::PID right_pid;
    double track_width;
    double wheels_diameter;
    double prev_left_vel, prev_right_vel;

    int get_closest(const Point& current_pose);
    double get_curvature(const Pose& robot, const Point& pt) const;
    void get_distances_each_pt();
    void get_curvature();
    std::optional<Point> get_lookahead_pt();
    Point circleLineIntersect(Point p1, Point p2);

    int prev_closest_index{0};
    double look_ahead_dist;
};
} // namespace voss::controller