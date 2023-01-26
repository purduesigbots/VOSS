#include "legs/chassis/distance_tracker.hpp"
#include "legs/errors.h"

namespace legs {

    LegsDistanceTracker::LegsDistanceTracker(unsigned int port_num, legs::distance_tracker_type type, bool reverse, double tpi) {
        
        port_num = port_num >=97 ? port_num - 96 : port_num >=65 ? port_num -64 : port_num; // :)
        
        this->port_num = port_num;
        this->type = type;
        this->tpi = tpi;

        if (type == LEGS_ROTATION) {
            if(!validate_port_smart(port_num)) {
                printf(LEGS_ERR_INVALID_ENCODER_PORT);
                return;
            }
            rot = std::make_shared<pros::Rotation>(port_num, port_num < 0);
            rot->reset();
        } else if (type == LEGS_ADI_ENCODER) {
            if(!validate_port_adi(port_num)) {
                printf(LEGS_ERR_INVALID_ENCODER_PORT);
                return;
            }
            enc = std::make_shared<pros::ADIEncoder>(port_num, port_num + 1, port_num < 0);
            enc->reset();
        } else {
            printf(LEGS_ERR_INVALID_ENCODER_TYPE);
            return;
        }
    }

    LegsDistanceTracker::LegsDistanceTracker(unsigned int port_num, int expander_port, bool reverse, double tpi) {
        
        port_num = port_num >=97 ? port_num - 96 : port_num >=65 ? port_num -64 : port_num; // :)
        
        this->port_num = port_num;
        this->type = LEGS_ADI_ENCODER;
        this->tpi = tpi;

        if(!validate_port_adi(port_num) || !validate_port_smart(expander_port)) {
            printf(LEGS_ERR_INVALID_ENCODER_PORT);
            return;
        }

        enc = std::make_shared<pros::ADIEncoder>(std::tuple<int, int, int>({expander_port, port_num, port_num + 1}), port_num < 0);
        enc->reset();
    }

    double LegsDistanceTracker::get_dist_traveled() {
        if(this->type == LEGS_ROTATION) {
            return rot->get_position() / this->tpi;
        } else if (this->type == LEGS_ADI_ENCODER) {
            return enc->get_value() / this->tpi;
        } else {
            return 0;
        }
    }

}