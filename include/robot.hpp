// ===================== robot.hpp =====================
#pragma once
#include "main.h"
#include <algorithm>
#include <cmath>

class Robot {
 public:
  Robot(int firstStagePort,
        int leverPort,
        int rotationPort,
        char blockerPort,
        char liftPort,
        char matchloaderPort,
        char wingPort);

  void init();

  // First stage (roller)
  void intakeOn();
  void intakeOff();
  void toggleIntake();
  void reverseIntake();

  // Lift piston
  void toggleLift();
  void raise();
  void lower();

  // Matchloader piston
  void matchloaderUp();
  void matchloaderDown();
  void toggleMatchloader();

  // Wing piston (cannot be up when lift is lowered)
  void wingUp();
  void wingDown();
  void toggleWing();

  // Lever move (NO PID)
  void moveLeverTo(double target_deg, int dir, int power = 80);

 private:
  // Hardware
  pros::Motor first_stage;
  pros::Motor lever;
  pros::Rotation lever_rotation;

  // Pistons
  pros::ADIDigitalOut blocker;
  pros::ADIDigitalOut lift;
  pros::ADIDigitalOut matchloader;
  pros::ADIDigitalOut wing;

  // State
  bool intake_running = false;
  bool lift_up = true;
  bool matchloader_up = false;
  bool wing_up = false;

  const bool piston_up_value = true;
  const bool piston_down_value = false;

  const bool blocker_open_value = true;
  const bool blocker_closed_value = false;

  // ===== Lever move state machine =====
  enum class LeverState {
    IDLE,
    MOVE_TO_TARGET
  };

  volatile LeverState lever_state = LeverState::IDLE;
  volatile bool lever_requested = false;

  double lever_target_deg = 0.0;
  int lever_dir = 1;
  int lever_power = 80;
  double lever_window = 10.0;

  static void lever_task_trampoline(void* param);
  void lever_task();
  pros::Task leverTask;

  void enforceWingRule();

  inline double lever_deg() {
    return lever_rotation.get_position() / 100.0; // centideg -> deg
  }
};
