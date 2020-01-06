#include "main.h"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "mtrs.hpp"
#include "automation_util.hpp"
#include "atoms.hpp"
#include "display.hpp"
#include "superhot_compat.hpp"

enum class MovementComponent { L, R };
inline MovementComponent invert(MovementComponent it) {
  if(it == MovementComponent::L) return MovementComponent::R;
  return MovementComponent::L;
}
struct PathDisturbance {
  MovementComponent dominant; // The side to use to measure progress during the disturbance.
  double targetLMax; // [0,1] The maximum L velocity drivable during the disturbance.
  double targetRMax; // [0,1] The maximum R v1elocity drivable during the disturbance.
  double activationDistance; //Absolute distance at which the PathDisturbance goes into effect.
  double lower; //Distance the disturbance is lowering L & R to their targets
  double sustain; //Distance the disturbance is keeping L & R at their targets 
  double resume; //Distance the disturbance is bringing back L & R to their original max velocities.
  bool used = false; //Internal flag, when a disturbance is used it can't be reused.
};

struct PIDOutput {
  virtual double getProgress() = 0;
  virtual double getAvgVel() = 0;
  virtual void controllerSet(double outVel) = 0;
  virtual ~PIDOutput() {}
};

static void doPID(double max, double revs, int time, bool shouldTurn, okapi::IterativePosPIDController ctrl, PIDOutput& out) {
  //Drive forward 20 revolutions.
  ctrl.setTarget(revs);
  ctrl.setOutputLimits(max, -max);
  auto endTime = pros::millis() + time;
  double lastVel = out.getAvgVel();
  double acc = 0;
  while(pros::millis() < endTime) {
    double step = ctrl.step(out.getProgress());
    out.controllerSet(step);
    double newVel = out.getAvgVel();
    acc = (newVel - lastVel) / 0.01;
    lastVel = newVel;
    pros::delay(10);
  }
  printf("%f fV, %f fE, %f acc on straight of %f\n", out.getAvgVel(), ctrl.getError(), acc, revs);
}

struct InterruptablePIDOutput: public PIDOutput {
  okapi::MotorGroup* L; double lOffset;
  okapi::MotorGroup* R; double rOffset;
  okapi::MotorGroup* D;//Dominant side
  int32_t startTime;
  PathDisturbance* activeDisturbance;
  std::vector<PathDisturbance> swerves;

  InterruptablePIDOutput(okapi::MotorGroup& l, okapi::MotorGroup& r, std::initializer_list<PathDisturbance> iswerves, double targetPos):
  L{&l}, lOffset{0}, R{&r}, rOffset{0}, D{L}, startTime{(int32_t)pros::millis()}, activeDisturbance{nullptr}, swerves{iswerves} {
    L->tarePosition(); R->tarePosition();
    //Negative activationDistances must be resolved with the targetPos.
    for(auto& it: swerves) {
      it.used = false;
      if(it.activationDistance < 0) {
        // on +: * * AD LOW SUS RES - - - - - -
        // on -: - - - - - - AD LOW SUS RES * * 
        it.activationDistance -= it.lower + it.sustain + it.resume;
        it.activationDistance += targetPos;
      }
    }
  }
  double getLPos() {
    return L->getPosition() + lOffset;
  }
  double getRPos() {
    return R->getPosition() + rOffset;
  }
  double getDPos() {
    if(D == L) return getLPos();
    return getRPos();
  }
  void setLPos(double rev) {
    lOffset = rev - L->getPosition();
  }
  void setRPos(double rev) {
    rOffset = rev - R->getPosition();
  }
  void highlightActiveDisturbance() {
    //First, check if the activeDisturbance is invalid.
    if(activeDisturbance) {
      auto &it = *activeDisturbance;
      if(it.activationDistance + it.lower + it.sustain + it.resume < getDPos()) {
        //All stages have passed. Time for this to go!
        activeDisturbance = nullptr;
        //Both sides should get the same position when there is no activeDisturbance.
        if(it.dominant == MovementComponent::L) {
          setRPos(getLPos());
        } else {
          setLPos(getRPos());
        }
      }
    }
    if(!activeDisturbance) {
      for(auto& it: swerves) {
        auto p = getDPos();
        if(!it.used && p >= it.activationDistance && p <= it.activationDistance + it.lower + it.sustain + it.resume) {
          activeDisturbance = &it;
          it.used = true;
          D = it.dominant == MovementComponent::L ? L : R;
        }
      }
    }
  }
  double getProgress() override {
    highlightActiveDisturbance();
    if(activeDisturbance) {
      return getDPos();
    } else {
      return (getLPos() + getRPos()) / 2;
    }
  }
  double interpolate(double target) {
    if(activeDisturbance) {
      //Check the current stage
      auto d = activeDisturbance->activationDistance + activeDisturbance->lower;
      auto p = getDPos();
      if(p < d) {
        auto delta = p - activeDisturbance->activationDistance;
        delta /= activeDisturbance->lower;
        //interpolate between (0, 1) to (1, target) with delta as x
        return (1 - delta) + delta * target;
      } else if(p < (d + activeDisturbance->sustain)) {
        return target;
      } else {
        auto delta = p - d - activeDisturbance->sustain;
        delta /= activeDisturbance->resume;
        return (1 - delta) * target + delta;
      }
    } else return 1;
  }
  double getLMax() {
    if(activeDisturbance) return interpolate(activeDisturbance->targetLMax);
    else return 1;
  }
  double getRMax() {
    if(activeDisturbance) return interpolate(activeDisturbance->targetRMax);
    else return 1;
  }
  double getAvgVel() override {
    return (std::abs(L->getActualVelocity()) / getLMax() + std::abs(R->getActualVelocity()) / getRMax()) / 2;
  }
  void controllerSet(double vel) override {
    highlightActiveDisturbance();
    L->controllerSet(vel * getLMax());
    R->controllerSet(vel * getRMax());
  }
};

static void straightNormal(double max, double revs, int time, std::initializer_list<PathDisturbance> iswerves = {}) {
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->right, iswerves, revs };
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.38,
    0.017,
    0.009,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void straightWeak(double max, double revs, int time, std::initializer_list<PathDisturbance> iswerves = {}) {
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->right, iswerves, revs };
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    0.6,
    0.011,
    0.004,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void turnNormal(double max, double revs, int time, std::initializer_list<PathDisturbance> iswerves = {}) {
  //Create PID controller from parameters in msg.
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->rightRev, iswerves, revs };
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    1.3,
    0.018,
    0.021,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void straightHeavy(double max, double revs, int time, std::initializer_list<PathDisturbance> iswerves = {}) {
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->right, iswerves, revs };
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.38,
    0.017,
    0.008,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void turnHeavy(double max, double revs, int time, std::initializer_list<PathDisturbance> iswerves = {}) {
  //Create PID controller from parameters in msg.
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->rightRev, iswerves, revs };
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    1.36,
    0.014,
    0.031,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

std::initializer_list<PathDisturbance> swerve(bool shouldInvert, double len = 0.5, double paddingDistance = 1) {
  auto d1 = MovementComponent::L; if(shouldInvert) d1 = invert(d1);
  auto d2 = invert(d1);
  return {
    {d1, shouldInvert ? 0.25 : 1, shouldInvert ? 1 : 0.25, +paddingDistance, 0.1, len, 0.1, false},
    {d2, shouldInvert ? 1 : 0.25, shouldInvert ? 0.25 : 1, -paddingDistance, 0.1, len, 0.1, false}
  };
}

static bool amBlue = true;
void setBlue(bool blue) { amBlue = blue; }
std::vector<std::string> grabAutonNames() {
  return {"nonprot", "prot", "skills", "selftest"};
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
  auto startTime = pros::millis();
  mtrs->tilter.tarePosition();
  mtrs->liftRaw.tarePosition();
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
      straightHeavy(0.8, w(-1.05) * -3.6, 2000);
      turnHeavy(0.8, w(1) * -0.22, 1000);
    }});
    parallel::waitForAll({[]{
      straightWeak(0.7, 2.6, 2000);
    }, []{
      pros::delay(200);
      mtrs->intake.controllerSet(1);
    }});
    turnHeavy(0.5, w(0.96) * 1, 1200);
    mtrs->intake.controllerSet(0.7);
    parallel::waitForAll({[]{
      pros::delay(600);
      mtrs->intake.controllerSet(0);
    }, []{
      straightHeavy(0.5, w(-0.87) * 3.77, 3000);
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
    straightNormal(0.6, 1.72, 2000);
    turnNormal(0.7, w(1.11) * -0.645, 1800);
    mtrs->intake.controllerSet(1);
    straightNormal(0.6, 2, 2000);
    parallel::waitForAll({[]{
      pros::delay(400);
      mtrs->intake.controllerSet(-1);
      pros::delay(280);
      mtrs->intake.controllerSet(0);
    }, []{
      turnNormal(0.7, w(1.3) * -0.3, 1000);
    }});
    straightNormal(0.8, w(-1.1) * 0.50, 1200);
    parallel::waitForAll({[&]{
      mtrs->tilter.moveAbsolute(4.3, 80);
      pros::delay(4800);
      //straightHeavy(1, 0.1, 800);
    }, [&] {
      pros::delay(1200);
      mtrs->intake.moveVoltage(-5000);
      pros::delay(300);
      mtrs->intake.controllerSet(0);
    }});
    straightNormal(0.8, -1, 1000);
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
    mtrs->lift.lowTarget();
    turnHeavy(0.6, 0.169, 2000);
    straightNormal(1, 0.1, 1500);
    //Double those points like ＼(^o^)／
    mtrs->intake.controllerSet(-1);
    pros::delay(1000);
  } else if(which == "selftest") {
    straightNormal(0.5, 3.5, 3000, swerve(true));
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
