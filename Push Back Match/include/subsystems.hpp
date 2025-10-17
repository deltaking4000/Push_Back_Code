#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup intake({-5, -6});  // Negative port will reverse the motor
inline ez::Piston matchloader('A');
 
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');