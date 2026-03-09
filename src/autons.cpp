#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(8.6, 0.0, 16.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(12.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.00, 20.0, 0);     // Turn in place constants
  chassis.pid_swing_constants_set(7.25, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(20_ms, 3_deg, 100_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 4_deg, 250_ms, 8_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(20_ms, 1_in, 100_ms, 4_in, 400_ms, 500_ms);
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

  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
}

void six_ball_right(){
  chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  wing.set(true);
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.pid_turn_set(295_deg, TURN_SPEED);
  pros::delay(200);
  intake.move(127);
  chassis.pid_drive_set(38_in, DRIVE_SPEED);
  pros::delay(900);
  chassis.pid_turn_set(345_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(15_in, 90);
  pros::delay(800);
  chassis.pid_drive_set(-15_in, 90);
  pros::delay(500);
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(-22_in, DRIVE_SPEED);
  pros::delay(500);
  chassis.pid_swing_set(LEFT_SWING, 90_deg, SWING_SPEED);
  pros::delay(500);
  chassis.drive_set(-50, -50);
  pros::Task scoreTask(score);
  pros::delay(800);
  wing.set(false);
  wing_toggle = false;
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  pros::delay(400);
  chassis.pid_drive_set(8_in, DRIVE_SPEED);
  pros::delay(500);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(300);
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  pros::delay(700);
  chassis.pid_turn_set(110_deg, 50);
}