#include "legs/chassis/distance_tracker.hpp"
#include "legs/errors.h"

namespace legs {

    LegsDistanceTracker::LegsDistanceTracker(unsigned int port_num, legs::distance_tracker_type type, bool reverse) {
        
        port_num = port_num >=97 ? port_num - 96 : port_num >=65 ? port_num -64 : port_num; // :)
        
        this->port_num = port_num;
        this->type = type;
        this->enc = nullptr;
        this->rot = nullptr;
        this->mot = nullptr;

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
            this->type = LEGS_INVALID_ENC_TYPE;
            return;
        }
    }

    //constructor used for internal motor encoders
    LegsDistanceTracker::LegsDistanceTracker(std::shared_ptr<pros::MotorGroup> m) {
        this->type = LEGS_MOT_ENCODER;
        this->mot = m;
    }

    double LegsDistanceTracker::get_dist_traveled() {
        if(this->type == LEGS_ROTATION) {
            if(rot==nullptr) {
                printf(LEGS_ERR_INVALID_ENCODER_PORT);
                return 0;
            }
            return rot->get_position() * this->tpi;
        } else if (this->type == LEGS_ADI_ENCODER) {
            if(enc==nullptr) {
                printf(LEGS_ERR_INVALID_ENCODER_PORT);
                return 0;
            }
            return enc->get_value() * this->tpi;
        } else if(this->type == LEGS_MOT_ENCODER) {
            if(mot==nullptr) {
                printf(LEGS_ERR_INVALID_ENCODER_PORT);
                return 0;
            }
            return mot->get_positions()[0];
        } else {
            //throw std::runtime_error(LEGS_ERR_INVALID_ENCODER_TYPE);
            printf(LEGS_ERR_INVALID_ENCODER_TYPE);
            return 0;
        }
    }

    void LegsDistanceTracker::set_tpi(double tpi) {
        this->tpi = tpi;
    }
}