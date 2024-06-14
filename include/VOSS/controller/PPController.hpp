
#pragma once
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/FeedFwd.hpp"
#include "VOSS/utils/PID.hpp"
namespace voss::controller {
class PPController : public AbstractController {
  public:
    struct Params {
        double lin_kp = 20;
        double lin_ki = 0;
        double lin_kd = 0;
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double look_ahead_dist = 5;
        double track_width = 14;
    };

    explicit PPController(Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

    void reset() override;

  private:
    utils::PID angular_pid;
    utils::PID linear_pid;
    double track_width;
    bool can_reverse = false;
    double min_error;

    int prev_closest_index{0};
    double look_ahead_dist;

    std::pair<double, double> pid_controller_for_ending(const Pose& current_pos,
                                                        const Pose& target,
                                                        bool reverse);

    int get_closest(const Pose& current_pose);
    double get_curvature(const Pose& robot, const Point& pt) const;
    std::optional<Point> get_lookahead_pt(const Pose& robot_pt, int idx);
    std::optional<Point> circle_line_intersect(const Pose& robot_pt,
                                               const Pose& start_pt,
                                               const Pose& end_pt) const;
    static double get_relative_y_error(const Pose& robot_pt, const Point& pt);
};
} // namespace voss::controller