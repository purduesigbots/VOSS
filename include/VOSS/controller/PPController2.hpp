#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"

namespace voss :: controller {
    class PPController2 : public AbstractController {
      public:
        struct Params {
            double lin_kp = 20;
            double lin_ki = 0;
            double lin_kd = 0;
            double ang_kp = 250;
            double ang_ki = 0;
            double ang_kd = 0;
            double look_ahead_dist;
            double track_width; //distance between sides of the drivtrain

        };

        explicit PPController2(Params params);

        static std::shared_ptr<PPController2> create_controller(Params params);

        chassis::DiffChassisCommand
        get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

        void reset() override;

        private:
            utils::PID angular_pid;
            utils::PID linear_pid;
            double track_width;
            bool can_reverse;
            double min_error;

            int prev_closest_index{0};
            double look_ahead_dist;

            std::pair<double, double> pid_controller_for_ending(const Pose&,
                                                                const Pose&,
                                                                bool);

            std::optional<Point> get_lookahead_pt(const Pose&, const int);
            int get_closest(const Pose&);
            std::optional<Point> circle_line_intersect(const Pose&,
                                                        const Pose&,
                                                        const Pose&) const;
            static Point get_relative_error(const Pose&, const Point&);
            
    };
} // namespace voss: controller