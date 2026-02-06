#include "EZ-Template/api.hpp"

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


// Pneumatics
void lift_matchloader() {
    matchloader.set(false); 
}

void drop_matchloader() {
    matchloader.set(true);
}

void lift_middlegoal() {
    middlegoal.set(false);
}

void drop_middlegoal() {
    middlegoal.set(true);
}

void lift_middlegoaldescore() {
    middlegoaldescore.set(false);
}

void drop_middlegoaldescore() {
    middlegoaldescore.set(true);
}

void lift_wingmech() {
    wingmech.set(true);
}

void drop_wingmech() {
    wingmech.set(false);
}
