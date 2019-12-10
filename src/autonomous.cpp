#include "main.h"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "mtrs.hpp"
#include "automation_util.hpp"
#include "atoms.hpp"
#include "display.hpp"
#include "superhot_compat.hpp"

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

static void straightWeak(double max, double revs, int time) {
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    0.6,
    0.011,
    0.004,
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
    0.018,
    0.021,
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
    1.36,
    0.014,
    0.031,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), mtrs->turn);
  mtrs->all.moveVoltage(0);
}

static bool amBlue = true;
void setBlue(bool blue) { amBlue = blue; }
std::vector<std::string> grabAutonNames() {
  return {"nonprot", "prot"};
}
std::string which = "nonprot";

//Runs a callable and copyable type, T, as a pros::Task.
//This really should be in its own file.
pros::Task runFuncAsync(const std::function<void()>& copyableRunnable) {
  auto copy = new std::function<void()>(copyableRunnable);
  return SuperHot::registerTask(pros::Task([](void* param) {
    auto innerCopy = new std::function<void()>(*((std::function<void()>*)param));
    delete (std::function<void()>*)param;
    innerCopy->operator()();
  }, copy, "runLambdaAsync"));
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

double w(double blueWeight) {
  return amBlue ? -blueWeight : 1;
}

double q(double redWeight) {
  return !amBlue ? -redWeight : 1;
}

void autonomous() {
  printf("hi\n");
  mtrs->all.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  mtrs->tilter.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  auto startTime = pros::millis();
  auto yeet = 5;
  mtrs->tilter.tarePosition();
  mtrs->lift.tarePosition();
  if(which == "nonprot") {
    mtrs->intake.controllerSet(1);
    straightWeak(0.7, 3.73, 2300);
    parallel::waitForAll({[]{
      pros::delay(600);
      mtrs->intake.controllerSet(-1);
      pros::delay(200);
      mtrs->intake.controllerSet(0);
    }, [] {
      turnHeavy(0.8, w(1) * 0.22, 1000);
      straightHeavy(0.8, -3.6, 2000);
      turnHeavy(0.8, w(1) * -0.22, 1000);
    }});
    mtrs->intake.controllerSet(1);
    straightWeak(0.7, 2.6, 2000);
    turnHeavy(0.5, w(1) * 0.94, 1200);
    mtrs->intake.controllerSet(0.7);
    parallel::waitForAll({[]{
      pros::delay(600);
      mtrs->intake.controllerSet(0);
    }, []{
      straightHeavy(0.5, w(-0.93) * 3.66, 3000);
    }, [&]{
      mtrs->tilter.moveAbsolute(2, 200);
      pros::delay(3000);
      mtrs->tilter.moveAbsolute(4.5, 200);
      pros::delay(1300);
      straightHeavy(1, w(-2) * 0.05, 800);
    }, [&]{
      pros::delay(2800);
      mtrs->intake.moveVoltage(-4000);
    }});
    mtrs->intake.controllerSet(0);
    straightHeavy(0.8, -1, 1000);
  } else if(which == "prot") {
    mtrs->intake.controllerSet(1);
    straightWeak(0.7, 1.7, 2000);
    parallel::waitForAll({[]{
      turnNormal(1, w(1) * -0.72, 1600);
    }, []{
      mtrs->intake.controllerSet(-1);
      pros::delay(300);
      mtrs->intake.controllerSet(0);
    }});
    mtrs->intake.controllerSet(1);
    straightWeak(0.7, 1.5, 2000);
    pros::delay(500);
    mtrs->intake.controllerSet(0);
    turnNormal(1, w(1) * -0.28, 2000);
    parallel::waitForAll({[&]{
      mtrs->tilter.moveAbsolute(2, 200);
      pros::delay(1600);
      mtrs->tilter.moveAbsolute(4.5, 200);
      //pros::delay(1500);
      //straightHeavy(1, 0.1, 800);
    }, [&]{
      pros::delay(1400);
      mtrs->intake.moveVoltage(-5000);
    }, [&]{
      straightHeavy(0.5, 1.0, 1600);
    }});
    straightHeavy(1, -1, 1000);
  } else if(which == "skills") {
    mtrs->intake.controllerSet(0.9);
    parallel::waitForAll({[&]{
      //
      straightHeavy(0.35, 8.1, 8000);
    }, [&]{
      pros::delay(2800);
      mtrs->intake.controllerSet(-0.9);
      pros::delay(220);
      mtrs->intake.controllerSet(0.9);
    }});
    pros::delay(200);
    turnHeavy(0.6, 0.435, 3000);
    parallel::waitForAll({[]{
      straightHeavy(0.5, 3.2, 3400);
    }, []{
      pros::delay(800);
      mtrs->intake.controllerSet(-1);
      pros::delay(260);
      mtrs->intake.controllerSet(0);
    }});
    parallel::waitForAll({[&]{
      mtrs->tilter.moveAbsolute(4.6, 80);
      pros::delay(4800);
      //straightHeavy(1, 0.1, 800);
    }});
    straightHeavy(0.5, -0.91, 2000);
    mtrs->tilter.moveAbsolute(0, 100);
    pros::delay(2000);
    turnNormal(1, 0.9, 2000);
    mtrs->intake.controllerSet(1);
    straightNormal(1, 2.8, 2800);
    pros::delay(800);
    mtrs->intake.controllerSet(-1);
    pros::delay(280);
    mtrs->intake.controllerSet(0);
    mtrs->lift.moveAbsolute(-4.4, 2000);
    turnHeavy(0.6, 0.169, 2000);
    straightNormal(1, 0.1, 1500);
    //Double those points like ＼(^o^)／
    mtrs->intake.controllerSet(-1);
    pros::delay(1000);
  }
  printf("Finished in %lums.\n", pros::millis() - startTime);
  auto remainderTime = 15000 + (int)startTime - (int)pros::millis();
  if(remainderTime > 0) pros::delay(remainderTime);
  printf("Time's up!\n");
  mtrs->all.moveVoltage(0);
  mtrs->intake.moveVoltage(0);
  mtrs->lift.moveVoltage(0);
  mtrs->tilter.moveAbsolute(0, 100);
  pros::delay(4000);
}

void r_initialize();
void initialize() {
  install_hawt_atom("auto", (void*)autonomous);
  install_hawt_atom("setBlue", (void*)setBlue);
  install_hawt_atom("grabAutonNames", (void*)grabAutonNames);
  r_initialize();
}
