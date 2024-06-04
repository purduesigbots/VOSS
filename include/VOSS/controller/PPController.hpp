
#pragma once
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/FeedFwd.hpp"
#include "VOSS/utils/PID.hpp"
namespace voss::controller {
class PPController : virtual public IsPathFollowController,
                     private std::enable_shared_from_this<PPController> {
  public:
    struct PP_Construct_Params {
        double lin_kp = 20;
        double lin_ki = 0;
        double lin_kd = 0;
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double look_ahead_dist = 5;
        double track_width = 14;
    };

    PPController(PP_Construct_Params params);


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
    utils::PID angular_pid;
    utils::PID linear_pid;
    double track_width;
    bool can_reverse = false;
    double min_error;

    int prev_closest_index{0};
    double look_ahead_dist;

    std::pair<double, double> pid_controller_for_ending(const Pose& current_pos, const Pose& target, bool reverse);

    int get_closest(const Pose& current_pose);
    double get_curvature(const Pose& robot, const Point& pt) const;
    std::optional<Point> get_lookahead_pt(const Pose& robot_pt, int idx);
    std::optional<Point> circle_line_intersect(const Pose& robot_pt, const Pose& start_pt, const Pose& end_pt) const;
    static double get_reference_y_err(const Pose& robot_pt, const Point& pt);
};
} // namespace voss::controller