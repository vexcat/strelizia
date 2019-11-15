#include "main.h"
#include "okapi/api.hpp"
#include "mtrs.hpp"
#include "automation_util.hpp"

static void doPID(double max, double revs, int time, bool shouldTurn, okapi::IterativePosPIDController ctrl, okapi::AbstractMotor& out) {
  //Drive forward 20 revolutions.
  out.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  out.tarePosition();
  ctrl.setTarget(revs);
  ctrl.setOutputLimits(max, -max);
  auto endTime = pros::millis() + time;
  double lastVel = out.getActualVelocity();
  double acc = 0;
  while(pros::millis() < endTime) {
    double step = ctrl.step(out.getPosition());
    out.moveVelocity(step * (int)out.getGearing());
    double newVel = out.getActualVelocity();
    acc = (newVel - lastVel) / 0.01;
    lastVel = newVel;
    pros::delay(10);
  }
  printf("%f fV, %f fE, %f acc on straight of %f\n", out.getActualVelocity(), ctrl.getError(), acc, revs);
}

static void straightPID(double max, double revs, int time) {
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.3,
    0.015,
    0.009,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), mtrs->all);
  mtrs->all.moveVoltage(0);
}

static void turnPID(double max, double revs, int time) {
  //Create PID controller from parameters in msg.
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    1.35,
    0.014,
    0.026,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), mtrs->turn);
  mtrs->all.moveVoltage(0);
}

static bool amBlue = false;
int which = 1;

//Runs a callable and copyable type, T, as a pros::Task.
//This really should be in its own file.
pros::Task runFuncAsync(const std::function<void()>& copyableRunnable) {
  auto copy = new std::function<void()>(copyableRunnable);
  return pros::Task([](void* param) {
    auto innerCopy = new std::function<void()>(*((std::function<void()>*)param));
    delete (std::function<void()>*)param;
    innerCopy->operator()();
  }, copy, "runLambdaAsync");
}

namespace parallel {
  void bruh(std::initializer_list<std::function<void()>> owo, bool anyFinish, bool cancelRemaining) {
    std::vector<pros::Task> tasks;
    for(const auto& func: owo) {
      tasks.push_back(runFuncAsync(func));
    }
    int initSize = tasks.size();
    while(anyFinish ? initSize == tasks.size() : tasks.size()) {
      tasks.erase(std::remove_if(tasks.begin(), tasks.end(), [](pros::Task& t) {
        return t.get_state() == TASK_STATE_DELETED;
      }), tasks.end());
      pros::delay(0);
    }
    if(!cancelRemaining) return;
    for(auto& task: tasks) {
      task.remove();
    }
  }
  void waitForAny(std::initializer_list<std::function<void()>> owo, bool cancelRemaining = false) {
    bruh(std::move(owo), true, cancelRemaining);
  }
  void waitForAll(std::initializer_list<std::function<void()>> owo) {
    bruh(std::move(owo), false, false);
  }
}

void whip() {
  // parallel::waitForAll({
  //   [] {
  //     straightPID(0.6, 0.6, 800);
  //     straightPID(0.8, -0.6, 800);
  //   },
  //   [] {
  //     int lockTarget = mtrs->lift.getPosition();
  //     mtrs->lift.controllerSet(-1);
  //     pros::delay(1400);
  //     mtrs->lift.moveAbsolute(lockTarget - 0.8, 100);
  //     pros::delay(800);
  //   }
  // });
  mtrs->intake.controllerSet(1);
  mtrs->lift.controllerSet(-1);
  pros::delay(1600);
  mtrs->intake.controllerSet(0);
  mtrs->lift.controllerSet(0);
}


void autonomous() {
  mtrs->all.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  auto startTime = pros::millis();
  if(which == 0) {
    whip();
    mtrs->intake.controllerSet(1);
    straightPID(0.6, 1.8, 900);
    for(int i = 0; i < 4; i ++) {
      straightPID(0.6, (4 - 1.8) / 4, 800);
    }
  }
  if(which == 1) {
    straightPID(0.7, -2.4, 2000);
    straightPID(0.7, 0.3, 2000);
    // turnPID(0.7, -1.25, 2000);
    // straightPID(0.7, 0.5, 2000);
    // whip();
  }
  if(which == 2) {
    whip();
  }
  if(which == 3) {
    whip();
    straightPID(0.6, 0.3, 2000);
    turnPID(0.6, -1.3, 2000);
    straightPID(0.6, -2, 3000);
  }
  printf("Finished in %lums.\n", pros::millis() - startTime);
  auto remainderTime = 15000 + startTime - pros::millis();
  if(remainderTime > 0) pros::delay(remainderTime);
  printf("Time's up!\n");
  mtrs->all.moveVoltage(0);
  mtrs->intake.moveVoltage(0);
  mtrs->lift.moveVoltage(0);
  mtrs->tilter.moveVoltage(0);
}

void r_initialize();
void initialize() {
  r_initialize();
  //autonomous(); 
}
