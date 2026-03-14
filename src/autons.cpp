#include "autons.hpp"

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "intake.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const int BACK_DRIVE_SPEED = 110;

void set_position() {
  chassis.drive_imu_reset(0);
  chassis.odom_xyt_set(0, 0, 0);
}

void set_position(double x, double y, double heading) {
  chassis.drive_imu_reset(heading);
  chassis.odom_xyt_set(x, y, heading);
}

///
// Constants
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
}

void tuned_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(16.3, 0.0, 124.5);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.1, 0.05, 27.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(5.5, 0.0, 68.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.7, 0.0, 105.0);   // Angular control for odom motions
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
  chassis.slew_drive_constants_set(3_in, 60);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  set_position();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  set_position();

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

// test to return to point (0,0,0) in odom
void odom_return_test() {
  chassis.pid_turn_set({0_in, 0_in}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0, TURN_SPEED);
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

  chassis.pid_odom_set(-12_in, BACK_DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, BACK_DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  set_position();
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  set_position();
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
  set_position();
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
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
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

void auton1() {
  set_position(0, 0, 0);
  intake_in();
  chassis.pid_odom_set({{{-5.77_in, 21.98_in}, fwd, DRIVE_SPEED},
                        {{-25.87_in, 39.24_in}, fwd, 70}},
                       true);
  chassis.pid_wait();
  pros::delay(1000);
  stopIntake();
  chassis.pid_odom_set({{{-16.53_in, 24.71_in}, rev, BACK_DRIVE_SPEED},
                        {{-34.41_in, 5.58_in}, rev, BACK_DRIVE_SPEED}},
                       true);
  chassis.pid_wait();
  chassis.pid_turn_set(178,TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-33.91, 23.14, 178}, rev, DRIVE_SPEED}, true);
  // chassis.pid_odom_set(-18_in,60);
  chassis.pid_wait();
  slopeScore();
  // set_position(-34.55,22.03, 180.88);
  pros::delay(3000);
  stopIntake();
  tongue.set(1);
  chassis.pid_odom_set(33.5_in, DRIVE_SPEED, true);
  pros::delay(200);
  intake_in();
  chassis.pid_wait();
  pros::delay(500);
  stopIntake();
  chassis.pid_odom_set(-27_in, 60, true);
  chassis.pid_wait_until(-7_in);
  tongue.set(0);
  chassis.pid_wait();
  slopeScore();
  pros::delay(900);
  stopIntake();
}

void auton2() {
  set_position(0, 0, 0);
  intake_in();
  chassis.pid_odom_set({{{5.77_in, 21.98_in}, fwd, DRIVE_SPEED},
                        {{25.87_in, 39.24_in}, fwd, 70}},
                       true);
  chassis.pid_wait();
  pros::delay(1000);
  stopIntake();
  chassis.pid_odom_set({{{16.53_in, 24.71_in}, rev, BACK_DRIVE_SPEED},
                        {{34.41_in, 5.58_in}, rev, BACK_DRIVE_SPEED}},
                       true);
  chassis.pid_wait();
  chassis.pid_turn_set(-178,TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{33.91, 23.14, -178}, rev, DRIVE_SPEED}, true);
  // chassis.pid_odom_set(-18_in,60);
  chassis.pid_wait();
  slopeScore();
  // set_position(-34.55,22.03, 180.88);
  pros::delay(3000);
  stopIntake();
  chassis.pid_odom_set(33_in, DRIVE_SPEED, true);
  tongue.set(1);
  pros::delay(200);
  intake_in();
  chassis.pid_wait();
  pros::delay(500);
  stopIntake();
  chassis.pid_odom_set(-27_in, 60, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(900);
  stopIntake();
}

void auton3() {
  set_position(0, 0, 0);
  intake_in();
  chassis.pid_odom_set({{{-5.77_in, 21.98_in}, fwd, DRIVE_SPEED},
                        {{-25.87_in, 39.24_in}, fwd, 70}},
                       true);
  chassis.pid_wait();
  pros::delay(1000);
  stopIntake();

  chassis.pid_odom_set({{-8.8, 26.15},rev,DRIVE_SPEED},true);
  chassis.pid_wait();
  chassis.pid_turn_set({2.49, 37.07}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{2.49, 37.07},rev,DRIVE_SPEED},true);
  chassis.pid_wait();
  middleScore();
  pros::delay(800);
  stopIntake();

  chassis.pid_odom_set({{-32.17,  3.81}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  chassis.pid_turn_set({-32.99, -6}, fwd, TURN_SPEED);
  chassis.pid_wait();
  tongue.set(1);
  pros::delay(500);
  intake_in();
  chassis.pid_odom_set({{-32.99, -6}, fwd, 127}, true);
  chassis.pid_wait();
  pros::delay(1000);
  stopIntake();
  chassis.pid_turn_set({-32.77, 22.06}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-32.77, 22.06}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(2000);
  // chassis.pid_odom_set({{{-16.53_in, 24.71_in}, rev, BACK_DRIVE_SPEED},
  //                       {{-34.41_in, 5.58_in}, rev, BACK_DRIVE_SPEED}},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(178,TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-33.91, 23.14, 178}, rev, DRIVE_SPEED}, true);
  // // chassis.pid_odom_set(-18_in,60);
  // chassis.pid_wait();
  // slopeScore();
  // // set_position(-34.55,22.03, 180.88);
  // pros::delay(3000);
  // stopIntake();
  // tongue.set(1);
  // pros::delay(200);
  // chassis.pid_odom_set(33.5_in, DRIVE_SPEED, true);
  // pros::delay(200);
  // intake_in();
  // chassis.pid_wait();
  // pros::delay(500);
  // stopIntake();
  // chassis.pid_odom_set(-27_in, 60, true);
  // chassis.pid_wait_until(-7_in);
  // tongue.set(0);
  // chassis.pid_wait();
  // slopeScore();
  // pros::delay(900);
  // stopIntake();
}

void auton4() {
  set_position(0, 0, 0);
  intake_in();
  chassis.pid_odom_set({{{-5.77_in, 21.98_in}, fwd, DRIVE_SPEED},
                        {{-25.87_in, 39.24_in}, fwd, 70}},
                       true);
  chassis.pid_wait();
  pros::delay(1000);
  stopIntake();

  chassis.pid_odom_set({{-8.8, 26.15},rev,DRIVE_SPEED},true);
  chassis.pid_wait();
  chassis.pid_turn_set({2.49, 37.07}, rev, TURN_SPEED);
  chassis.pid_wait();
  // chassis.pid_odom_set({{2.49, 37.07},rev,DRIVE_SPEED},true);
  // chassis.pid_wait();
  // middleScore();
  // pros::delay(800);
  // stopIntake();

  chassis.pid_odom_set({{-32.17,  3.81}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  chassis.pid_turn_set({-32.99, -5.5}, fwd, TURN_SPEED);
  chassis.pid_wait();
  tongue.set(1);
  pros::delay(500);
  intake_in();
  chassis.pid_odom_set({{-32.99, -6}, fwd, 127}, true);
  chassis.pid_wait();
  pros::delay(1000);
  stopIntake();
  chassis.pid_turn_set({-34.27, 21.06}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-34.27, 21.06}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(2000);
}

void autonSkills(){
  set_position(-48.28, -15.424, 180);
  chassis.pid_odom_set({{{-48.093, -47.838}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  chassis.pid_turn_set({-57.836, -47.276}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{{-57.836, -47.276}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  

  chassis.pid_odom_set({{{-34.432, -58.796}, fwd, DRIVE_SPEED}, {{34.484, -58.796}, rev, DRIVE_SPEED}}, true);
}

void fast_wing_auton(){
  set_position(-46.64, 13.833, 90);
  intake_in();
  chassis.pid_odom_set({{{-23.086_in, 19.523_in}, fwd, DRIVE_SPEED},
                        {{-45.64_in, 47.965_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait_until_index_started(0);
  tongue.set(1);
  chassis.pid_wait();
  chassis.pid_turn_set({-60.194_in, 47.965_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-22.546, 47.965}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(1600);
  intake_in();
  tongue.set(0);

  chassis.pid_odom_set({{-39.546_in, 58.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, 58.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-8.546_in, 58.465_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);
}

void right_fast_wing_auton(){
  set_position(-46.64, -13.833, 90);
  intake_in();
  chassis.pid_odom_set({{{-21.086_in, -19.523_in}, fwd, DRIVE_SPEED},
                        {{-45.64_in, -47.965_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait_until_index_started(0);
  pros::delay(20);
  tongue.set(1);
  chassis.pid_wait();
  chassis.pid_turn_set({-60.194_in, -47.965_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-26.546, -47.965}, rev, DRIVE_SPEED}, true);
  // tongue.set(0);
  // chassis.pid_wait_until(22);
  chassis.pid_wait();
  slopeScore();
  pros::delay(1400);
  intake_in();

  chassis.pid_odom_set({{-39.546_in, -37.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-8.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);


}


void wing_auton(){
  set_position(-46.64, 13.833, 90);
  intake_in();
  chassis.pid_odom_set({{{-23.086_in, 19.523_in}, fwd, DRIVE_SPEED},
                        {{-45.64_in, 47.965_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait_until_index_started(0);
  tongue.set(1);
  chassis.pid_wait();
  chassis.pid_turn_set({-56.194_in, 48.965_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-56.194_in, 48.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(500);

  chassis.pid_odom_set({{-26.546, 47.965}, rev, DRIVE_SPEED}, true);
  pros::delay(100);
  tongue.set(0);
  chassis.pid_wait();
  intake_out();
  pros::delay(100);
  slopeScore();
  pros::delay(1700);
  intake_in();

  chassis.pid_odom_set({{-39.546_in, 58.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, 58.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-8.546_in, 58.465_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);
}

void right_wing_auton(){
  set_position(-46.64, -13.833, 90);
  intake_in();
  chassis.pid_odom_set({{{-21.086_in, -19.523_in}, fwd, DRIVE_SPEED},
                        {{-45.64_in, -47.965_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait_until_index_started(0);
  pros::delay(20);
  tongue.set(1);
  chassis.pid_wait();
  chassis.pid_turn_set({-56.194_in, -47.965_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-56.194_in, -47.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(500);

  chassis.pid_odom_set({{-27.546, -47.965}, rev, DRIVE_SPEED}, true);
  pros::delay(100);
  tongue.set(0);
  chassis.pid_wait();
  intake_out();
  pros::delay(100);
  slopeScore();
  pros::delay(1700);
  intake_in();

  chassis.pid_odom_set({{-39.546_in, -37.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-8.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);
}

void right_bottom_auton(){
  set_position(-46.64, -13.833, 90);
  intake_in();
  chassis.pid_odom_set({{{-21.086_in, -19.523_in}, fwd, DRIVE_SPEED},
                        {{-45.64_in, -47.965_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait_until_index_started(0);
  pros::delay(20);
  tongue.set(1);
  chassis.pid_wait();

  chassis.pid_turn_set({5.494_in, 9.119_in}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-5.494, 9.119}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  
  // chassis.pid_turn_set({-56.194_in, -47.965_in}, fwd, DRIVE_SPEED);
  // chassis.pid_wait();
  chassis.pid_odom_set({{-56.194_in, -47.965_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(500);

  chassis.pid_odom_set({{-27.546, -47.965}, rev, DRIVE_SPEED}, true);
  pros::delay(100);
  tongue.set(0);
  chassis.pid_wait();
  intake_out();
  pros::delay(100);
  slopeScore();
  pros::delay(1700);
  intake_in();

  chassis.pid_odom_set({{-39.546_in, -37.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-8.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);
}

void sawp(){
  // first loader and goal
  set_position(-48.28, -15.424, 180);
  tongue.set(1);
  chassis.pid_odom_set({{-48.28, -49.838}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  intake_in();
  chassis.pid_turn_set({-56.6,-49.838}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-56.6, -49.838}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_odom_set({{-26.417, -49.838}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait_quick();
  slopeScore();
  pros::delay(1100);
  stopIntake();
  tongue.set(0);

  // two clusters and middle
  chassis.pid_odom_set({{-39.417, -47.838}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  intake_in();
  chassis.pid_turn_set({-22.237,-22.544}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{{-22.237_in, -22.544_in}, fwd, DRIVE_SPEED},
                        {{-22.237_in, 22.544_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  // pros::delay(200);
  // tongue.set(1);
  // pros::delay(300);
  // tongue.set(0);
  chassis.pid_wait();
  chassis.pid_turn_set({-10.556,10.556}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-10.556, 10.556}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait();
  middleScore();
  pros::delay(500);
  stopIntake();

  // second loader and goal
  tongue.set(1);
  chassis.pid_odom_set({{-48.28, 47.838}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  intake_in();
  chassis.pid_turn_set({-56.6,47.838}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-56.6, 47.838}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_odom_set({{-26.417, 47.838}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait_quick();
  slopeScore();
  pros::delay(1100);
  stopIntake();
  tongue.set(0);
}

void auton_skills(){
  // first loader and goal
  set_position(-48.28, -15.424, 180);
  tongue.set(1);
  chassis.pid_odom_set({{-48.28, -47.638}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  intake_in();
  chassis.pid_turn_set({-57.1,-47.638}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-57.1, -47.638}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(1200);

  chassis.pid_odom_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, -59.465_in}, rev, BACK_DRIVE_SPEED},
                        {{30.546_in, -59.465_in}, rev, BACK_DRIVE_SPEED},
                        {{40.546_in, -47.638_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait();

  // 2nd loader

  chassis.pid_turn_set({57.1,-47.638}, fwd, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{26.417, -47.838}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(1750);
  intake_in();
  chassis.pid_odom_set({{57.1, -47.638}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(1200);

  chassis.pid_odom_set({{26.417, -47.838}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(1750);
  intake_in();

  // 3rd loader

  chassis.pid_odom_set({{40.546, 47.838}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  chassis.pid_turn_set({57.1,-47.638}, fwd, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{57.1, -47.638}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(1200);

  // 4th loader
  chassis.pid_odom_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{{30.546_in, 59.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-30.546_in, 59.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-40.546_in, 47.638_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set({-57.1,47.638}, fwd, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-26.417, 47.838}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(1750);
  intake_in();
  chassis.pid_odom_set({{-57.1, 47.638}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(1200);

  chassis.pid_odom_set({{-26.417, 47.838}, rev, BACK_DRIVE_SPEED}, true);
  chassis.pid_wait();
  slopeScore();
  pros::delay(1750);
  intake_in();

  chassis.pid_odom_set({{-63, 0}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();


  // chassis.pid_odom_set({{-45.546_in, -47.965_in}, fwd, DRIVE_SPEED}, true);
  // chassis.pid_wait();

  // chassis.pid_odom_set({{{-30.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
  //                       {{-8.546_in, -37.465_in}, rev, BACK_DRIVE_SPEED},
  //                      },
  //                      true);
  
}

void middle_auton(){
  set_position(-46.64, 13.833, 90);
  intake_in();
  chassis.pid_odom_set({{{-23.086_in, 21.523_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  pros::delay(550);
  tongue.set(1);
  chassis.pid_wait();

  //score on middle goal
  chassis.pid_turn_set({-5.494_in, 9.119_in}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-5.494, 9.119}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
  middleScore();
  pros::delay(750);
  intake_in();


  chassis.pid_odom_set({
                        {{-44.64_in, 47.965_in}, fwd, DRIVE_SPEED},
                       },
                       true);
  chassis.pid_wait();
  chassis.pid_turn_set({-56.194_in, 48.965_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{-56.194_in, 48.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  pros::delay(400);

  chassis.pid_odom_set({{-26.546, 48.965}, rev, DRIVE_SPEED}, true);
  pros::delay(100);
  tongue.set(0);
  chassis.pid_wait();
  intake_out();
  pros::delay(100);
  slopeScore();
  pros::delay(1600);
  intake_in();

  chassis.pid_odom_set({{-39.546_in, 58.965_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-30.546_in, 58.465_in}, rev, BACK_DRIVE_SPEED},
                        {{-8.546_in, 58.465_in}, rev, BACK_DRIVE_SPEED},
                       },
                       true);

}
