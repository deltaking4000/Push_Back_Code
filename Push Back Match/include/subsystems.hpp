#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup intake({-5});  // Negative port will reverse the motor
inline ez::Piston matchloader('A');
inline pros::Motor lowerintake(-11);  // Negative port will reverse the motor
inline pros::Motor upperintake(-12);  // Negative port will reverse the motor
inline pros::Motor outtake(-6);
inline ez::Piston wingmech('B');
 
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');