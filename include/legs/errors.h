
#pragma once

#include "legs/api.hpp"


#define LEGS_ERR_INVALID_ENCODER_PORT "Invalid Encoder Port"
#define LEGS_ERR_INVALID_ENCODER_TYPE "Invalid Encoder Type"


double validate_port_smart(int port);
double validate_port_adi(int port);