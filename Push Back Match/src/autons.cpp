#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int DRIVE_SPEED_SLOW = 70;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;


///
// Constantsq
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there

  chassis.drive_imu_scaler_set(360.0/359.28);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED_SLOW, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  intake.move(-127);
  matchloader.set(true);
  chassis.odom_xyt_set(46_in, 7.5_in, 0_deg);    // Set the current position, you can start at a specific position with this
/*
  chassis.pid_odom_set({{46_in, 46.928_in, 90_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
*/

  // Path

  chassis.pid_odom_set({{46.095_in, 46.928_in, 90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(2 * 1000); 
  intake.move(0);
  chassis.pid_drive_set(-6_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  matchloader.set(false);
  chassis.pid_odom_set({{33.92_in, 34.475_in, 270_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{-47.843_in, 34.475_in, 270_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{-31.867_in, 47.454_in, 270_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();


  // // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  // chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
  //                       {{0_in, 20_in}, fwd, DRIVE_SPEED},
  //                       {{0_in, 30_in}, fwd, DRIVE_SPEED}},
  //                      true);
  // chassis.pid_wait();

  // // Drive to 0, 0 backwards
  // chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
  //                      true);
  // chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void SpooktacularAutonRight() {

  lowerintake.move(-127);
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(25_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(137_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED);
  chassis.pid_wait();
  upperintake.move(-127);
}

void SpooktacularAutonLeft() {
  chassis.odom_theta_flip();
  SpooktacularAutonRight();
}

///
// 15 second auton with matchloader - robot starts above parkzone facing right
///
void AutonRightMatchloaderOnly() {
  // rightmatchloader
  chassis.odom_xyt_set(46_in, 7.5_in, 0_deg);    // Set the current position, you can start at a specific position with this

  // start intake and drop matchloader
  drop_matchloader();

  // drive to matchloader
  chassis.pid_drive_set(39.344_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED_SLOW); 
  chassis.pid_wait();
  spin_intake();
  pros::delay(2 * 1000);
  chassis.pid_drive_set(-1_in, DRIVE_SPEED); 
  chassis.pid_wait();
  pros::delay(0.5 * 1000);
  chassis.pid_drive_set(1_in, DRIVE_SPEED_SLOW); 
  chassis.pid_wait();
  pros::delay(0.5 * 1000);
  stop_intake(); 
  

  // wait couple seconds to remove blocks from matchloader
  spin_intake();
  pros::delay(2 * 1000);
  stop_intake(); 

  // backup into long goal
  chassis.pid_drive_set(-29.711_in, DRIVE_SPEED); 
  chassis.pid_wait();

  // score into long goal
  spin_intake();
  spin_outtake();
  pros::delay(4 * 1000); 

  // all done
  lift_matchloader();
}

void AutonLeftMatchloaderOnly() {
  chassis.odom_theta_flip();
  AutonRightMatchloaderOnly();
}

///
// 60 second auton - robot starts on right of parkzone
///
void SpooktacularAuton60Seconds() {
  // only start lower intake so preload don't get thrown out 
  lowerintake.move(-127);

  // drive to/pickup 4-blocks on right
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(25_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  // drive to long goal
  chassis.pid_turn_set(137_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED);
  chassis.pid_wait();

  // score in long goal and wait for blocks to score
  upperintake.move(-127);
  pros::delay(5 * 1000); 
  intake.move(0);

  // park the robot
  chassis.pid_drive_set(37_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(267_deg, TURN_SPEED);

  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
}


///
// 60 second auton - robot starts above parkzone facing right
///
void PressureBreakpointSkills() {

  matchloader.set(true);
  spin_intake();

  // 1. 1st match-loader - drive and intake

 
  chassis.pid_drive_set(41, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10.5_in, DRIVE_SPEED_SLOW); 
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  // wait for intake to complete
  pros::delay(0.5 * 1000); 
  // JIGGLE
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();


  // 2. drive to other end of long-goal andcore
  chassis.pid_drive_set(-3.411_in, DRIVE_SPEED); 
  chassis.pid_wait();
  stop_intake();


  matchloader.set(false);

  chassis.pid_turn_set(124.737_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-22_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-65.066_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(57.923_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-27_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-27_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  // score in long goal and wait for blocks to score
  spin_intake();
  spin_outtake();
  pros::delay(5 * 1000); 
  outtake.move(0);

  // 3. 2nd match-loader intake and score
  matchloader.set(true);

  chassis.pid_drive_set(34.5_in, DRIVE_SPEED_SLOW); 
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  // wait for intake to complete
  
  // JIGGLE
  pros::delay(0.5 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  stop_intake();

  chassis.pid_drive_set(-35_in, 55); 
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  // score in long goal and wait for blocks to score
  spin_intake();
  spin_outtake();
  pros::delay(5 * 1000); 
  outtake.move(0);
  matchloader.set(false);
  chassis.pid_drive_set(3_in, DRIVE_SPEED); 
  chassis.pid_wait();
 chassis.pid_drive_set(-3_in, DRIVE_SPEED); 
  chassis.pid_wait();

// afterparty

chassis.pid_drive_set(10, DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(100, DRIVE_SPEED);
chassis.pid_wait();
  
  // Conclusion of Act 1 -180
  // Part 2

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10.5_in, DRIVE_SPEED_SLOW); 
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  // wait for intake to complete
  pros::delay(0.5 * 1000); 
  // JIGGLE
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  pros::delay(0.25 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();


  // 2. drive to other end of long-goal andcore
  chassis.pid_drive_set(-3.411_in, DRIVE_SPEED); 
  chassis.pid_wait();
  stop_intake();


  matchloader.set(false);

  chassis.pid_turn_set(-124.737_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-22_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-65.066_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(-57.923_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-33_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(-270_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-27_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-270_deg, TURN_SPEED);
  chassis.pid_wait();

  // score in long goal and wait for blocks to score
  spin_intake();
  spin_outtake();
  pros::delay(5 * 1000); 
  outtake.move(0);

  // 3. 2nd match-loader intake and score
  matchloader.set(true);

  chassis.pid_drive_set(34.5_in, DRIVE_SPEED_SLOW); 
  chassis.pid_wait();
  chassis.pid_turn_set(-270_deg, TURN_SPEED);
  chassis.pid_wait();

  // wait for intake to complete
  
  // JIGGLE
  pros::delay(0.5 * 1000); 
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(-2.5_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED_SLOW);
  chassis.pid_wait();
  stop_intake();

  chassis.pid_drive_set(-35_in, 55); 
  chassis.pid_wait();
  chassis.pid_turn_set(-270_deg, TURN_SPEED);
  chassis.pid_wait();

  // score in long goal and wait for blocks to score
  spin_intake();
  spin_outtake();
  pros::delay(5 * 1000); 
  outtake.move(0);
  matchloader.set(false);
  chassis.pid_drive_set(3_in, DRIVE_SPEED); 
  chassis.pid_wait();
 chassis.pid_drive_set(-3_in, DRIVE_SPEED); 
  chassis.pid_wait();

// park
chassis.pid_drive_set(24, DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-0_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(55, DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-90_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(48, DRIVE_SPEED);
chassis.pid_wait();

}

///
// 60 second auton - robot starts above parkzone facing right
///
void SpaceAuton60Seconds() {
  // rightmatchloader
  // SuperSoloAWP

drop_matchloader();
spin_intake();

// first matchload
chassis.pid_drive_set(39.344_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(90.738_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(12.905_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(92_deg, TURN_SPEED);
pros::delay(1 * 1000);
chassis.pid_turn_set(90_deg, TURN_SPEED);
pros::delay(1 * 1000);
chassis.pid_turn_set(88_deg, TURN_SPEED);
pros::delay(1 * 1000);
chassis.pid_wait();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait();
stop_intake();
chassis.pid_drive_set(-29.711_in, DRIVE_SPEED); 
chassis.pid_wait();
spin_intake();
spin_outtake();
pros::delay(2 * 1000); 
lift_matchloader();
chassis.pid_turn_set(88.453_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(6.159_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(107.879_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(16.261_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(113.552_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(7.08_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(167.039_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(21.516_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(178.898_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(23_in, DRIVE_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait();


}

//
// 60 second auton - robot starts above parkzone facing right
///
void SuperSoloAWP() {
  // SuperSoloAWP
  chassis.odom_xyt_set(46_in, 7.5_in, 180_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_angle_set(180_deg);    // Set the current angle

  // take care of the other robot and steal their preload
  intake.move(-127);
  chassis.pid_drive_set(7.916_in, DRIVE_SPEED); 
  chassis.pid_wait();

  // drive to matchloader to matchload
  chassis.pid_drive_set(-35_in, DRIVE_SPEED); ///////////////////////////////////////////////
  chassis.pid_wait_quick_chain();
  drop_matchloader();
  chassis.pid_turn_set(93_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  // intaking from the loader
  chassis.pid_drive_set(12_in, DRIVE_SPEED, false); 
  chassis.pid_wait();
  pros::delay(0.1*1000); // pick up alliance color blocks from loader

  // score into 1st long goal
  chassis.pid_drive_set(-28.7_in, DRIVE_SPEED); 
  chassis.pid_wait();
  outtake.move(-127);
  pros::delay(1.5*1000); // score into long goal
  outtake.move(0);
  lift_matchloader();

  // go get 3-block heap #1
  chassis.pid_swing_set(ez::LEFT_SWING, 230_deg, SWING_SPEED, ez::RIGHT_TURN);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(16_in, DRIVE_SPEED); 
  chassis.pid_wait_until(7_in);
  drop_matchloader();
  chassis.pid_wait_quick_chain();
 
  // go get 3-block heap #2
  chassis.pid_turn_set(190_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  lift_matchloader();
  
  chassis.pid_odom_set({{22_in, -32_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  // scoring in middle goal
  chassis.pid_turn_set(140_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED); 
  chassis.pid_wait_quick_chain();
  drop_middlegoal();
  pros::delay(0.5*1000); // score middle goal blocks - leave some for long goal
  stop_outtake();
  lift_middlegoal();
  
  // Last long goal
  chassis.pid_turn_set(135.544_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(49.541_in, DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-18.807_in, DRIVE_SPEED); 
  chassis.pid_wait();
  outtake.move(-127); // dump all blocks
  pros::delay(1.5*1000); // score long goal

}

void Move_Nothing_Auton(){
  // do nothing

}

void MoveFwd1() {
  // SuperSoloAWP
  chassis.pid_drive_set(1_in, DRIVE_SPEED);

}

void Left_7_ball() {
  chassis.odom_theta_flip();
  Right_7_ball();

}

void Right_7_ball() {

// 7-ball w/o wing
chassis.odom_xyt_set(46.2_in, 14.8_in, 0_deg);    // Set the current position, you can start at a specific position with this
//  chassis.drive_angle_set(0_deg);    // Set the current angle for drive motions

spin_intake();
chassis.pid_turn_set(340_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(26_in, DRIVE_SPEED_SLOW); 
chassis.pid_wait_until(15_in);
drop_matchloader();
chassis.pid_wait();
chassis.pid_turn_set(46.538_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-36_in, DRIVE_SPEED);  
chassis.pid_wait();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(8.5_in, DRIVE_SPEED); 
chassis.pid_wait();
pros::delay(0.75 * 1000); 
chassis.pid_drive_set(-32.213_in, DRIVE_SPEED); 
chassis.pid_wait();
lift_matchloader();
spin_outtake();
pros::delay(2 * 1000); 

// dancing in spedtember

// the ram is annoying


}



// Important Stuff
//pros::delay(2 * 1000); 
