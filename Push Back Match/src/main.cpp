
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
///// 



// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, 15, -17},     // Left Chassis Ports (negative port will reverse it!)
    {10, -16, 18},  // Right Chassis Ports (negative port will reverse it!)

    7,      // Gyro Port
    (24.0/17.91)*3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450,   // Wheel RPM = cartridge * (motor gear / wheel gear) 

    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 84/36 which is 2.333
    // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 36/60 which is 0.6
    48.0/36.0);

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels
// ez::tracking_wheel horiz_tracker(-14, 2, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(-12, 2, 0.4);  // This tracking wheel is parallel to the drive wheels

//////////////////////////////////////////////////////////
// Lemlib config:
/*
// motor groups
pros::MotorGroup leftMotors({-1, 15, -17}, pros::MotorGearset::blue); // left motor group 
pros::MotorGroup rightMotors({10, -16, 18}, pros::MotorGearset::blue); // right motor group  

// Inertial Sensor on port 10
pros::Imu imu(7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis + traction
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis lemchassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
*/
//////////////////////////////////////////////////////////


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  //  chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  // chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      // {"RIGHT BLUE/RED (BASIC)\nPosition on the right.", SpooktacularAutonRight},
      // {"LEFT BLUE/RED (BASIC)\nPosition on the left.", SpooktacularAutonLeft},
      {"doesnt move\n\nPosition at parkzone corner facing either direction", Move_Nothing_Auton},
      {"Moves fwd 1 inch\n\nPosition at parkzone corner facing either direction", MoveFwd1},
      {"RIGHT BLUE/RED WITH MATCHLOADER\nPosition above parkzone facing right.", AutonRightMatchloaderOnly},
      {"LEFT BLUE/RED WITH MATCHLOADER\nPosition above parkzone facing left.", AutonLeftMatchloaderOnly},
      {"LEFT SIDE 7 BALL\nPosition above parkzone facing up.", Right_7_ball},
      {"RIGHT SIDE 7 BALL\nPosition above parkzone facing up.", Left_7_ball},
      {"SuperSoloAWP\n\nPosition above parkzone facing right", SuperSoloAWP},
      {"60 SECOND AUTON\nPosition above park zone facing left on the right side.",  PressureBreakpointSkills},
      // {"Drive\n\nDrive forward and come back", drive_example},
      // {"Odom Drive\n\nDrive forward and come back", odom_drive_example},   Right_7_ball
      // {"Odom Coordinates PP Drive\n\nDrive forward and come back", odom_pure_pursuit_example},
      // {"Turn\n\nTurn 3 times.", turn_example},
      // {"Tracking Wheel Offsets\n\n Measure Tracking Wheel Offsets.", measure_offsets},

      /*
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
      */
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  //////////////////////////////////////////////////////////////////
  // lemchassis.calibrate(false); // calibrate sensors
  /////////////////////////////////////////////////////////////////
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
      
      if (ez::as::page_blank_is_on(1)) {
          ez::screen_print("Left: " + util::to_string_with_precision(chassis.drive_sensor_left()) +
          "\nRight: " + util::to_string_with_precision(chassis.drive_sensor_right()), 1);
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    // if (master.get_digital_new_press(DIGITAL_X))
    //   chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

// Curvature drive from lemlib 
// https://github.com/LemLib/LemLib/blob/79acbe657559d08057f5de5136761ad88ce49bae/src/lemlib/chassis/opcontrol.cpp#L27
void opcontrol_curvature(int forward, int turn) {
    // If we're not moving forwards change to arcade drive
    if (forward == 0) {
      chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
      return;
    }

    double leftPower = forward + std::abs(forward) * turn;
    double rightPower = forward - std::abs(forward) * turn;

    chassis.drive_set(leftPower, rightPower);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  bool intakeStarted = false; 
  bool enableCurvatureDrive = false;

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    //////////////////////////////////////////////////////////////////////////////
    // Use lemlib opcontrol for curvature drive instead of eztemplate
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    if (master.get_digital(DIGITAL_Y) && master.get_digital(DIGITAL_DOWN)) {
      enableCurvatureDrive = !enableCurvatureDrive;
      master.print(0, 0, enableCurvatureDrive ? "Curvature Drive" : "Arcade Drive   ");
    }

    if (enableCurvatureDrive) {
      int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      opcontrol_curvature(leftY, rightX);
    } else {
      chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    }
    //////////////////////////////////////////////////////////////////////////////

    // . . .
    // Put more user control code here!
    // . . .

/*
		if (master.get_digital_new_press(DIGITAL_R1)) {
			if (intakeStarted == false) {
				intake.move_velocity(600);
				intakeStarted = true;
			}
			else {
				intake.move_velocity(0);
				intakeStarted = false;
			}
		}

		else if (master.get_digital_new_press(DIGITAL_R2)) {

			if (intakeStarted == false) {
				intake.move_velocity(-600);
				intakeStarted = true;
			}
			else {
				intake.move_velocity(0);
				intakeStarted = false;
			}
		}*/

     // Uncomment for hold-button to spin, comment out above block

    if (master.get_digital(DIGITAL_R1)) {
      intake.move(-127);
    } 
    else if (master.get_digital(DIGITAL_R2)){
      outtake.move(-127);
      intake.move(-127);
    }
    else if (master.get_digital(DIGITAL_UP)) {
      intake.move(127);
    } 
    else {
      outtake.move(0);
      intake.move(0);
    }

    if (master.get_digital(DIGITAL_L1)) {
      matchloader.set(true);
    } 
    else if (master.get_digital(DIGITAL_L2)) {
      matchloader.set(false);
    } 




    if (master.get_digital(DIGITAL_X)) {
      wingmech.set(true);
    } 
    if (master.get_digital(DIGITAL_B)) {
      wingmech.set(false);
         
    }

    middlegoal.button_toggle(master.get_digital(DIGITAL_A));
  
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
