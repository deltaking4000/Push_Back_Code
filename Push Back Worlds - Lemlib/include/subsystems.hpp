#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup intake({-5});  // Negative port will reverse the motor
inline pros::Motor lowerintake(-11);  // Negative port will reverse the motor
inline pros::Motor upperintake(-12);  // Negative port will reverse the motor
inline pros::Motor outtake(-14);
// inline ez::Piston matchloader('A');
// inline ez::Piston middlegoaldescore('G');
// inline ez::Piston wingmech('B');
// inline ez::Piston middlegoal('H');

#include "pros/adi.hpp"


pros::adi::Pneumatics matchloader('A', true); 
pros::adi::DigitalOut middlegoaldescore('G'); 
pros::adi::DigitalOut wingmech('B'); 
pros::adi::DigitalOut middlegoal('H'); 


void spin_intake();
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