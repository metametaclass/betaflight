#pragma once

#include "sitl2_state.h"

//set random imu values for debugging
void sitl2_set_random_imu();

int sitl2_calc_physics(sitl2_state_t* state);
