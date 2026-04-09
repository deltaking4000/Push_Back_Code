#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Motors
inline pros::MotorGroup intake({-5});  
inline pros::Motor outtake(-14);
inline pros::Motor lowerintake(-11);  // not used now
inline pros::Motor upperintake(-12);  // not used now

// Pneumatics
inline ez::Piston matchloader('A');
inline ez::Piston middlegoaldescore('G');
inline ez::Piston wingmech('B');
inline ez::Piston middlegoal('H');

// Sensors
inline pros::Optical optical(11);

void spin_intake();
void spin_intake_slow();
void spin_outtake();
void stop_intake();
void stop_outtake();
void reverse_intake();
void reverse_outtake();
void lift_matchloader();
void drop_matchloader();
void lift_middlegoal();
void drop_middlegoal();
void lift_middlegoaldescore();
void drop_middlegoaldescore();
void lift_wingmech();
void drop_wingmech();