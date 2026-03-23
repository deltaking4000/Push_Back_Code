#include "subsystems.hpp"

// Intakes and Outtakes
void spin_intake() {
    intake.move(-127);
}

void spin_outtake() {
    outtake.move(-127);
}

void stop_intake() {
    intake.move(0);
}

void stop_outtake() {
    outtake.move(0);
}

void reverse_intake() {
    intake.move(127);
}

void reverse_outtake(){
    outtake.move(127);
}


// // Pneumatics
void lift_matchloader() {
    matchloader.set_value(false);  // retract
}

void drop_matchloader() {
   matchloader.set_value(true);   // extend
}

void lift_middlegoal() {
    middlegoal.set_value(true);  // retract
    
}

void drop_middlegoal() {
    middlegoal.set_value(false);  // extend
   
}

void lift_middlegoaldescore() {
    middlegoaldescore.set_value(true);
}

void drop_middlegoaldescore() {
    middlegoaldescore.set_value(false);
}

void lift_wingmech() {
    wingmech.set_value(true);
}

void drop_wingmech() {
    wingmech.set_value(false);
}
