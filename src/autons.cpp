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
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
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

void score_mid_auto(){
    intake.move(127);
    lever.move_velocity(80);
    blocker.set(true);
    pros::delay(1000);
    lever.move(-127);
    pros::delay(400);
    lever.move_velocity(0);
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void six_ball_starter(){
    chassis.odom_xyt_set(48_in, 11_in, 270_deg);
  wing.set(true);
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.pid_turn_set(295_deg, TURN_SPEED);
  pros::delay(200);
  intake.move(127);
  chassis.pid_drive_set(38_in, DRIVE_SPEED);
  pros::delay(100);
  matchloader.set(true);
  pros::delay(900);
  chassis.pid_turn_set(345_deg, TURN_SPEED);
  pros::delay(500);
  matchloader.set(false);
  chassis.pid_drive_set(15_in, 90);
  pros::delay(200);
  matchloader.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-17_in, 90);
  pros::delay(700);
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  pros::delay(600);
  matchloader.set(false);
  chassis.pid_drive_set(-19_in, 127);
  pros::delay(500);
  chassis.pid_swing_set(LEFT_SWING, 90_deg, SWING_SPEED, -10);
  pros::delay(500);
  chassis.drive_set(-127, -127);
  pros::delay(200);
  pros::Task scoreTask(score);
}

void six_ball_right_wing(){
  six_ball_starter();
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

void six_ball_right_score(){
  six_ball_starter();
  pros::delay(3000);
  pros::Task scoreTask2(score);
}


void sawp(){
  chassis.odom_xyt_set(48_in, 11_in, 0_deg);
  wing.set(true);
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  intake.move(127);
  matchloader.set(true);
  chassis.pid_drive_set(32.5_in, DRIVE_SPEED);
  pros::delay(900);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(15_in, 80);
  pros::delay(1000);
  chassis.pid_drive_set(-30_in, DRIVE_SPEED);
  pros::delay(1500);
  chassis.drive_set(-127, -127);
  matchloader.set(false);
  pros::Task scoreTask(score);
  pros::delay(800);
  chassis.pid_turn_set(210_deg, TURN_SPEED);
  pros::delay(900);
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  matchloader.set(true);
  pros::delay(500);
  pros::delay(200);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  pros::delay(500);
  matchloader.set(false);
  lift.set(true);
  lift_toggle = true;
  chassis.pid_drive_set(46_in, DRIVE_SPEED);
  pros::delay(400);
  matchloader.set(true);
  pros::delay(800);
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  pros::delay(700);
  score_mid_auto();
  chassis.pid_drive_set(50_in, DRIVE_SPEED);
  pros::delay(500);
  lift.set(false);
  lift_toggle = false;
  pros::delay(500);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(15_in, 80);
  pros::delay(1000);
  chassis.pid_drive_set(-30_in, DRIVE_SPEED);
  pros::delay(1500);
  chassis.drive_set(-127, -127);
  matchloader.set(false);
  pros::Task scoreTask2(score);
}

void nine_ball_right_wing(){
  six_ball_starter();
  matchloader.set(true);
  pros::delay(1200);
  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  pros::delay(1500);
  chassis.pid_drive_set(-11_in, DRIVE_SPEED);
  pros::delay(700);
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  matchloader.set(false);
  pros::delay(500);
  chassis.pid_drive_set(51_in, DRIVE_SPEED);
  pros::delay(1400);
  intake.move(-127);
  pros::delay(1000);
}