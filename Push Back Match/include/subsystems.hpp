#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup intake({-5});  // Negative port will reverse the motor
inline pros::Motor lowerintake(-11);  // Negative port will reverse the motor
inline pros::Motor upperintake(-12);  // Negative port will reverse the motor
inline pros::Motor outtake(-8);
inline ez::Piston matchloader('A');
inline ez::Piston wingmech('B');
inline ez::Piston middlegoal('H');

void spin_intake();
void spin_outtake();
void stop_intake();
void stop_outtake();
void reverse_intake();

void lift_matchloader();
void drop_matchloader();
void lift_middlegoal();
void drop_middlegoal();
void lift_wingmech();
void drop_wingmech();