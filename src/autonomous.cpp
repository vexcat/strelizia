#include "main.h"
#include "okapi/api.hpp"
#include "mtrs.hpp"
#include "automation_util.hpp"
#include "atoms.hpp"

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

static void straightNormal(double max, double revs, int time) {
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.38,
    0.017,
    0.009,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), mtrs->all);
  mtrs->all.moveVoltage(0);
}

static void turnNormal(double max, double revs, int time) {
  //Create PID controller from parameters in msg.
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    1.3,
    0.017,
    0.029,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), mtrs->turn);
  mtrs->all.moveVoltage(0);
}

static void straightHeavy(double max, double revs, int time) {
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.38,
    0.017,
    0.008,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), mtrs->all);
  mtrs->all.moveVoltage(0);
}

static void turnHeavy(double max, double revs, int time) {
  //Create PID controller from parameters in msg.
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    1.2,
    0.026,
    0.029,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), mtrs->turn);
  mtrs->all.moveVoltage(0);
}

static bool amBlue = false;
int which = 0;

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
  mtrs->tilter.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  auto startTime = pros::millis();
  if(which == 0) {
    parallel::waitForAll({[&]{
      straightNormal(0.8, 2.5, 1400);
      straightNormal(1, -2, 1300);
    }, [&] {
      pros::delay(200);
      mtrs->intake.controllerSet(1);
      pros::delay(2000);
      mtrs->intake.controllerSet(0);
    }});
    turnHeavy(0.6, 0.68, 950);
    straightHeavy(1, 2, 1200);
    turnHeavy(0.6, -0.67, 950);
    mtrs->intake.controllerSet(1);
    straightHeavy(0.5, 2.9, 2000);
    parallel::waitForAll({[&]{
      straightHeavy(0.5, -1.6, 1400);
    }, [&]{
      pros::delay(800);
      mtrs->intake.controllerSet(0);
    }});
    turnHeavy(0.5, 1.3, 700);
    parallel::waitForAll({[&]{
      straightHeavy(1, 1.7, 1000);
      mtrs->tilter.moveRelative(3.98, 200);
      pros::delay(3400);
    }, [&]{
      pros::delay(900);
      mtrs->intake.moveVoltage(-4000);
    }});
    //YEET
    straightHeavy(0.6, -1, 1000);
  }
  printf("Finished in %lums.\n", pros::millis() - startTime);
  auto remainderTime = 15000 + (int)startTime - (int)pros::millis();
  if(remainderTime > 0) pros::delay(remainderTime);
  printf("Time's up!\n");
  mtrs->all.moveVoltage(0);
  mtrs->intake.moveVoltage(0);
  mtrs->lift.moveVoltage(0);
  mtrs->tilter.moveVoltage(0);
}

void r_initialize();
void initialize() {
  install_hawt_atom("auto", (void*)autonomous);
  r_initialize();
}
