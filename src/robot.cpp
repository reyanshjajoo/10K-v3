#include "robot.hpp"

#include <cmath>

Robot::Robot(int firstStagePort,
             int leverPort,
             int rotationPort,
             char blockerPort,
             char liftPort,
             char matchloaderPort,
             char wingPort)
    : first_stage(firstStagePort),
      lever(leverPort),
      lever_rotation(rotationPort),
      blocker(blockerPort),
      lift(liftPort),
      matchloader(matchloaderPort),
      wing(wingPort),
      scoreTask(Robot::score_task_trampoline, this) {
  // Initial piston states
  // Lift requirement from earlier: starts UP
  lift.set_value(piston_up_value);
  lift_up = true;

  // Blocker (stopper): closed by default
  blocker.set_value(blocker_closed_value);

  // New pistons: both default DOWN
  matchloader.set_value(piston_down_value);
  wing.set_value(piston_down_value);
  matchloader_up = false;
  wing_up = false;
}

void Robot::enforceWingRule() {
  // Wing is uncontrollable when lift is lowered: force down
  if (!lift_up) {
    wing.set_value(piston_down_value);
    wing_up = false;
  }
}

void Robot::init() {
  // Default piston states (defaults up)
  lift.set_value(piston_up_value);
  lift_up = true;

  blocker.set_value(blocker_closed_value);

  matchloader.set_value(piston_down_value);
  matchloader_up = false;

  wing.set_value(piston_down_value);
  wing_up = false;

  // Make sure wing rule is enforced when lift is lowered
  enforceWingRule();

  lever.tare_position();

  // Safety: stop motors at init
  first_stage.move(0);
  lever.move(0);
  intake_running = false;

  lever_rotation.reset_position();
  lever.set_brake_mode(MOTOR_BRAKE_HOLD);
  lever.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

// -------- First stage --------

void Robot::intakeOn() {
  intake_running = true;
  first_stage.move(127);
}

void Robot::intakeOff() {
  intake_running = false;
  first_stage.move(0);
}

void Robot::toggleIntake() {
  intake_running = !intake_running;
  first_stage.move(intake_running ? 127 : 0);
}

void Robot::reverseIntake() {
  first_stage.move(-127);
}

// -------- Lift piston --------

void Robot::raise() {
  lift.set_value(piston_up_value);
  lift_up = true;
  // Now wing is allowed again (but stays whatever it currently is).
}

void Robot::lower() {
  lift.set_value(piston_down_value);
  lift_up = false;

  // Force wing down when lift is lowered
  enforceWingRule();
}

void Robot::toggleLift() {
  if (lift_up)
    lower();
  else
    raise();
}

// -------- Matchloader piston --------

void Robot::matchloaderUp() {
  matchloader.set_value(piston_up_value);
  matchloader_up = true;
}

void Robot::matchloaderDown() {
  matchloader.set_value(piston_down_value);
  matchloader_up = false;
}

void Robot::toggleMatchloader() {
  if (matchloader_up)
    matchloaderDown();
  else
    matchloaderUp();
}

// -------- Wing piston (restricted by lift) --------

void Robot::wingUp() {
  // If lift is lowered, wing cannot go up
  enforceWingRule();
  if (!lift_up) return;

  wing.set_value(piston_up_value);
  wing_up = true;
}

void Robot::wingDown() {
  wing.set_value(piston_down_value);
  wing_up = false;
}

void Robot::toggleWing() {
  // If lift is lowered, wing is forced down and cannot toggle up
  enforceWingRule();
  if (!lift_up) return;

  if (wing_up)
    wingDown();
  else
    wingUp();
}

// -------- Lever score sequence (async) --------

void Robot::score() {
  // Only accept a new request if we're idle
  if (score_state != ScoreState::IDLE) return;

  // Choose speed cap based on current lift position
  active_lever_speed = lift_up ? lever_full_speed : lever_slow_speed;

  score_requested = true;
}

void Robot::score_task_trampoline(void* param) {
  static_cast<Robot*>(param)->score_task();
}

void Robot::score_task() {
  while (true) {
    if (score_requested && score_state == ScoreState::IDLE) {
      score_requested = false;

      blocker.set_value(blocker_open_value);
      state_start_ms = pros::millis();
      score_state = ScoreState::OPEN_BLOCKER_DELAY;
    }

    switch (score_state) {
      case ScoreState::IDLE:
        break;

      case ScoreState::OPEN_BLOCKER_DELAY:
        if (pros::millis() - state_start_ms >= blocker_open_delay_ms) {
          lever.move_absolute(
              lever_score_position,
              active_lever_speed);

          score_state = ScoreState::MOVE_TO_SCORE;
        }
        break;

      case ScoreState::MOVE_TO_SCORE:
        if (std::abs(lever.get_position() - lever_score_position) < lever_settle_window_deg) {
          state_start_ms = pros::millis();
          score_state = ScoreState::HOLD_SCORE;
        }
        break;

      case ScoreState::HOLD_SCORE:
        if (pros::millis() - state_start_ms >= lever_hold_ms) {
          lever.move_absolute(
              lever_home_position,
              active_lever_speed);

          score_state = ScoreState::MOVE_TO_HOME;
        }
        break;

      case ScoreState::MOVE_TO_HOME:
        if (std::abs(lever.get_position() - lever_home_position) < lever_settle_window_deg) {
          blocker.set_value(blocker_closed_value);
          score_state = ScoreState::IDLE;
        }
        break;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}