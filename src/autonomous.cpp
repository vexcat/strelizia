#include "main.h"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "mtrs.hpp"
#include "automation_util.hpp"
#include "atoms.hpp"
#include "display.hpp"
#include "superhot_compat.hpp"
#include "hotdb.hpp"

enum class MovementComponent { L, R };
inline MovementComponent invert(MovementComponent it) {
  if(it == MovementComponent::L) return MovementComponent::R;
  return MovementComponent::L;
}
struct PathDisturbance {
  MovementComponent dominant; // The side to use to measure progress during the disturbance.
  double targetLMax; // [0,1] The maximum L velocity drivable during the disturbance.
  double targetRMax; // [0,1] The maximum R velocity drivable during the disturbance.
  double activationDistance; //Absolute distance at which the PathDisturbance goes into effect.
  double lower; //Distance the disturbance is lowering L & R to their targets
  double sustain; //Distance the disturbance is keeping L & R at their targets 
  double resume; //Distance the disturbance is bringing back L & R to their original max velocities.
  bool used = false; //Internal flag, when a disturbance is used it can't be reused.
  void dump() {
    printf("%d,%f,%f,%f,%f,%f,%f,%d\n", (int)dominant, targetLMax, targetRMax, activationDistance, lower, sustain, resume, (int)resume);
  }
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
  printf("%f fV, %f fE, %f acc on PID of %f\n", out.getAvgVel(), ctrl.getError(), acc, revs);
}

struct InterruptablePIDOutput: public PIDOutput {
  okapi::MotorGroup* L; double lOffset;
  okapi::MotorGroup* R; double rOffset;
  okapi::MotorGroup* D;//Dominant side
  int32_t startTime;
  PathDisturbance* activeDisturbance;
  std::vector<PathDisturbance> swerves;

  InterruptablePIDOutput(okapi::MotorGroup& l, okapi::MotorGroup& r, std::vector<PathDisturbance> iswerves, double targetPos):
  L{&l}, lOffset{0}, R{&r}, rOffset{0}, D{L}, startTime{(int32_t)pros::millis()}, activeDisturbance{nullptr}, swerves{iswerves} {
    L->tarePosition(); R->tarePosition();
    targetPos = std::abs(targetPos);
    //Negative activationDistances must be resolved with the targetPos.
    for(auto& it: swerves) {
      it.dump();
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
    //printf("lOffset was updated\n");
  }
  void setRPos(double rev) {
    rOffset = rev - R->getPosition();
    //printf("rOffset was updated\n");
  }
  void highlightActiveDisturbance() {
    //First, check if the activeDisturbance is invalid.
    if(activeDisturbance) {
      auto &it = *activeDisturbance;
      if(it.activationDistance + it.lower + it.sustain + it.resume < std::abs(getDPos())) {
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
    //Find a new activeDisturbance
    if(!activeDisturbance) {
      for(auto& it: swerves) {
        auto p = std::abs(getDPos());
        //printf("DPos @ %f, L @ %f, R @ %f\n", p, getLPos(), getRPos());
        if(!it.used && p >= it.activationDistance && p <= it.activationDistance + it.lower + it.sustain + it.resume) {
          activeDisturbance = &it;
          //printf("activeDisturbance was set\n");
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
      auto p = std::abs(getDPos());
      if(p < d) {
        auto delta = p - activeDisturbance->activationDistance;
        delta /= activeDisturbance->lower;
        //interpolate between (0, 1) to (1, target) with delta as x
        //printf("Lower %f\n", delta);
        return (1 - delta) + delta * target;
      } else if(p < (d + activeDisturbance->sustain)) {
        //printf("Sustain\n");
        return target;
      } else {
        auto delta = p - d - activeDisturbance->sustain;
        delta /= activeDisturbance->resume;
        return (1 - delta) * target + delta;
        //printf("Resume %f\n", delta);
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
    return (std::abs(L->getActualVelocity()) / std::abs(getLMax()) + std::abs(R->getActualVelocity()) / std::abs(getRMax())) / 2;
  }
  void controllerSet(double vel) override {
    highlightActiveDisturbance();
    L->controllerSet(vel * getLMax());
    R->controllerSet(vel * getRMax());
  }
};

double absIMUStart;
class EpicTurn: public PIDOutput {
  okapi::MotorGroup* mtrCaptive;
  pros::Imu* imuCaptive;
  int i = 0;
  public:
  EpicTurn(okapi::MotorGroup& out, pros::Imu& sensor): mtrCaptive{&out}, imuCaptive{&sensor} {}
  double getProgress() override {
    auto ret = imuCaptive->get_rotation() - absIMUStart;
    i++;
    if(i % 10 == 0) printf("%f\n", ret);
    return ret;
  }
  double getAvgVel() override {
    return mtrCaptive->getActualVelocity();
  }
  void controllerSet(double outVel) override {
    mtrCaptive->controllerSet(outVel);
  }
};

static void straightNormal(double max, double revs, int time, std::vector<PathDisturbance> iswerves = {}) {
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->right, iswerves, revs };
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.38,
    0.017,
    0.009,
    0,
    okapi::TimeUtilFactory::createDefault(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void straightWeak(double max, double revs, int time, std::vector<PathDisturbance> iswerves = {}) {
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->right, iswerves, revs };
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    0.6,
    0.011,
    0.004,
    0,
    okapi::TimeUtilFactory::createDefault(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void turnNormal(double max, double revs, int time, std::vector<PathDisturbance> iswerves = {}) {
  //Create PID controller from parameters in msg.
  //InterruptablePIDOutput pidOut{ mtrs->left, mtrs->rightRev, iswerves, revs };
  EpicTurn pidOut{ mtrs->turn, *imuPtr };
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    0.007,
    0.00005,
    0.00031,
    0,
    okapi::TimeUtilFactory::createDefault(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void turnMini(double max, double revs, int time, std::vector<PathDisturbance> iswerves = {}) {
  EpicTurn pidOut{ mtrs->turn, *imuPtr };
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    *((double*)retrieve_hawt_atom("miniP")),
    *((double*)retrieve_hawt_atom("miniI")),
    *((double*)retrieve_hawt_atom("miniD")),
    0,
    okapi::TimeUtilFactory::createDefault(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void straightHeavy(double max, double revs, int time, std::vector<PathDisturbance> iswerves = {}) {
  InterruptablePIDOutput pidOut{ mtrs->left, mtrs->right, iswerves, revs };
  doPID(max, revs, time, false, okapi::IterativePosPIDController(
    1.38,
    0.017,
    0.008,
    0,
    okapi::TimeUtilFactory::createDefault(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

static void turnHeavy(double max, double revs, int time, std::vector<PathDisturbance> iswerves = {}) {
  //Create PID controller from parameters in msg.
  EpicTurn pidOut{ mtrs->turn, *imuPtr };
  //InterruptablePIDOutput pidOut{ mtrs->left, mtrs->rightRev, iswerves, revs };
  doPID(max, revs, time, true, okapi::IterativePosPIDController(
    0.6,
    0.000,
    0.038,
    0,
    okapi::TimeUtilFactory::createDefault(),
    std::make_unique<okapi::AverageFilter<3>>()
  ), pidOut);
  mtrs->all.moveVoltage(0);
}

std::vector<PathDisturbance> swerve(bool shouldInvert, double len = 0.4, double paddingDistance = 0.4, double ramp = 0.1) {
  auto d1 = MovementComponent::L; if(shouldInvert) d1 = invert(d1);
  auto d2 = invert(d1);
  return {
    {d1, shouldInvert ? -0.5 : 1, shouldInvert ? 1 : -0.5, +paddingDistance, ramp, len, ramp, false},
    {d2, shouldInvert ? 1 : -0.5, shouldInvert ? -0.5 : 1, -paddingDistance, ramp, len, ramp, false}
  };
}

static bool amBlue = true;
void setBlue(bool blue) { amBlue = blue; }
std::vector<std::string> grabAutonNames() {
  return {"selftest", "nonprot", "3prot", "4prot", "skills", "whip", "6"};
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
  void bruh(std::vector<std::function<void()>> owo, bool anyFinish, bool cancelRemaining) {
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

void waitIMUStabilize() {
  auto startTime = pros::millis();
  auto lastStableTime = pros::millis();
  bool stableAck = false;
  auto iniATT = imuPtr->get_euler();
  while(true) {
    auto acc = imuPtr->get_accel();
    auto att = imuPtr->get_euler();
    if(std::sqrt(acc.x*acc.x + acc.y*acc.y) > 0.7 || std::abs(att.pitch - iniATT.pitch) > 12 || std::abs(att.yaw - iniATT.yaw) > 12) {
      stableAck = false;
    } else {
      if(!stableAck) {
        stableAck = true;
        lastStableTime = pros::millis();
      } else if(pros::millis() > lastStableTime + 1000) {
        printf("Stabilized after %dms\n", pros::millis() - startTime);
        return;
      }
    }
    if(pros::millis() > startTime + 4000) {
      printf("Early exit after %dms\n", pros::millis() - startTime);
      return;
    }
    pros::delay(10);
  }
}

void whipout() {
  mtrs->tilter.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  pros::delay(100);
  mtrs->intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  parallel::waitForAll({[]{
    mtrs->intake.moveRelative(-0.15, 100);
    pros::delay(650);
    pros::delay(300);
    mtrs->lift.moveVoltage(6000);
    pros::delay(800);
    mtrs->lift.controllerSet(0);
    mtrs->intake.controllerSet(0);
  }, []{
    mtrs->all.tarePosition();
    mtrs->all.moveAbsolute(0 + 0.5, 100);
    pros::delay(1200);
    mtrs->all.moveVoltage(-9500);
    pros::delay(1200);
  }});
  mtrs->tilter.moveAbsolute(0, 100);
  mtrs->liftRaw.moveAbsolute(0, 100);
  mtrs->tilter.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  mtrs->intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

std::function<void()> wagarms(double multiplier, double per = 100.0) {
  return [=]{
    auto startTime = pros::millis();
    while(true) {
      auto target = 0.5 - (std::cos((pros::millis() - startTime) / per) / 2.0);
      mtrs->liftRaw.moveAbsolute(-target * multiplier, 100);
      pros::delay(10);
    }
  };
}

PathDisturbance w(PathDisturbance unmirrored) {
  if(amBlue) {
    unmirrored.dominant = invert(unmirrored.dominant);
    double oldL = unmirrored.targetLMax;
    unmirrored.targetLMax = unmirrored.targetRMax;
    unmirrored.targetRMax = oldL;
  }
  return unmirrored;
}

void trayFailSafe() {
  //The tray tends to lose its position after whipout().
  //To counteract this for safe operation in the rest of auton and opcontrol, this method
  //gently lowers the tray until trayButton is pressed (or until a timeout of 3s), then
  //tares the motor.
  auto endTime = pros::millis() + 3000; // 3s
  mtrs->tilter.moveVoltage(-3000); // 1/4 pwr
  while(!trayBumper->get_value() && pros::millis() < endTime) {
    pros::delay(10);
  }
  mtrs->tilter.tarePosition();
}

void autonomous() {
  mtrs->intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  which = getSelectedAuton();
  auto startTime = pros::millis();
  mtrs->tilter.tarePosition();
  mtrs->liftRaw.tarePosition();
  absIMUStart = imuPtr->get_rotation();
  // Default / Test auton
  if(which == "selftest") which = "skills";
  if(which == "nonprot") {
    whipout();
    mtrs->intake.controllerSet(1);
    parallel::waitForAny({[]{
      parallel::waitForAll({[]{
        straightNormal(0.3, 3.7, 4000, {
          w({MovementComponent::R, 0.4, 1, -0.6, 0.05, 0.5, 0.05, false})
        });
      }, trayFailSafe});
    }, wagarms(1)}, true);
    mtrs->liftRaw.moveAbsolute(0, 100);
    turnNormal(0.5, w(1) * 155, 1800);
    mtrs->intake.controllerSet(0);
    //mtrs->tilter.moveAbsolute(1.323, 95);
    mtrs->tilter.moveAbsolute(0.8, 95);
    straightNormal(0.4, 3.2, 3500);
    //mtrs->tilter.moveAbsolute(3.349, 80);
    mtrs->tilter.moveAbsolute(3, 80);
    pros::delay(1800);
    straightNormal(0.4, -1, 1000);
  } else if(which == "3prot") {
    whipout();
    mtrs->intake.controllerSet(1);
    parallel::waitForAll({[]{
      straightNormal(0.4, 1.7, 1500);
      turnNormal(0.4, w(1) * -90, 2000);
    }, trayFailSafe});
    straightNormal(0.4, 1.4, 1000);
    mtrs->tilter.moveAbsolute(0.8, 95);
    mtrs->intake.controllerSet(0);
    turnNormal(0.4, w(1) * -130, 2000);
    straightNormal(0.4, 1.2, 1500);
    mtrs->tilter.moveAbsolute(3, 60);
    pros::delay(2600);
    mtrs->tilter.moveAbsolute(0, 80);
    straightNormal(0.4, -1, 1000);
  } else if(which == "4prot") {
    whipout();
    mtrs->intake.controllerSet(1);
    parallel::waitForAll({[]{
      straightNormal(0.4, 1.7, 1500);
      turnNormal(0.4, w(1) * -90, 2000);
    }, trayFailSafe});
    straightNormal(0.4, 1.4, 1000);
    mtrs->tilter.moveAbsolute(0.8, 95);
    mtrs->intake.controllerSet(0);
    turnNormal(0.4, w(1) * -130, 2000);
    straightNormal(0.4, 1.2, 1500);
    mtrs->tilter.moveAbsolute(3, 60);
    pros::delay(2600);
    mtrs->tilter.moveAbsolute(0, 80);
    straightNormal(0.4, -1, 1000);
  } else if(which == "6") {
    whipout();
    mtrs->intake.controllerSet(1);
    //parallel::waitForAny({[]{
    //}, wagarms(1)}, true);
    parallel::waitForAll({[]{
      straightNormal(0.3, 3.7, 4000, {
        w({MovementComponent::R, 0.4, 1, -0.6, 0.05, 0.5, 0.05, false})
      });
    }, trayFailSafe});
    mtrs->liftRaw.moveAbsolute(0, 100);
    turnNormal(0.5, w(1) * 0, 800);
    straightNormal(0.5, -2.1, 1200);
    turnNormal(0.5, w(1) * 135, 1800);
    mtrs->intake.controllerSet(0);
    //mtrs->tilter.moveAbsolute(1.323, 95);
    mtrs->tilter.moveAbsolute(1, 95);
    straightNormal(0.4, 1.4, 1700);
    //mtrs->tilter.moveAbsolute(3.349, 80);
    mtrs->tilter.moveAbsolute(2.6, 80);
    auto score1Start = pros::millis();
    while(mtrs->tilter.getPosition() < 1.571) {
      pros::delay(10);
    }
    mtrs->tilter.moveAbsolute(2.6, 50);
    auto score1Delta = 2700 - (pros::millis() - score1Start);
    if(score1Delta >= 0)
      pros::delay(score1Delta);
    straightNormal(0.4, -1, 1000);
  } else if(which == "skills") {
    //absIMUStart -= 270;
    //goto mouii;
    {
    mtrs->intake.controllerSet(1);
    //parallel::waitForAny({[]{
    //}, wagarms(1)}, true);
    parallel::waitForAll({[]{
      straightNormal(0.28, 3.8, 4200, {
        w({MovementComponent::R, 0.4, 1, -0.6, 0.05, 0.5, 0.05, false})
      });
    }, trayFailSafe});
    mtrs->liftRaw.moveAbsolute(0, 100);
    turnMini(1.0, w(1) * 0, 800);
    straightNormal(0.8, -1.7, 1200);
    //mtrs->intake.moveRelative(-0.2, 80);
    turnMini(1.0, w(1) * 134, 1400);
    mtrs->intake.controllerSet(0);
    //mtrs->tilter.moveAbsolute(1.323, 95);
    straightNormal(0.6, 1.7, 1800);
    //mtrs->tilter.moveAbsolute(3.349, 80);
    mtrs->tilter.moveAbsolute(2.8, 80);
    auto score1Start = pros::millis();
    while(mtrs->tilter.getPosition() < 1.00) {
      pros::delay(10);
    }
    mtrs->tilter.moveAbsolute(2.8, 38);
    auto score1Delta = 4000 - (pros::millis() - score1Start);
    if(score1Delta >= 0)
      pros::delay(score1Delta);
    straightNormal(0.4, 0.2, 800);
    straightNormal(0.30, -1, 1000);
    mtrs->tilter.moveAbsolute(0, 100);
    turnNormal(1.0, 270, 2000);
    //ram into wall
    mtrs->all.moveVoltage(-5000);
    pros::delay(500);
    while(std::abs(mtrs->all.getActualVelocity()) > 50) pros::delay(10);
    pros::delay(600);
    }
mouii:
    mtrs->intake.controllerSet(1);
    straightNormal(0.5, 3.6, 4000);
    mtrs->intake.moveRelative(-0.3, 100);
    pros::delay(200);
    straightNormal(0.5, -0.41, 1100); //kizu darake
    mtrs->tilter.moveAbsolute(1.2, 100);
    mtrs->liftRaw.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
    mtrs->liftRaw.moveAbsolute(-3.2, 80); // -3.070
    pros::delay(2600);
    parallel::waitForAll({[]{
      //straightNormal(0.3, 0.1, 800);
    }, []{
      mtrs->intake.controllerSet(-1);
      pros::delay(1000);
      //straightNormal(0.3, -0.1, 800);
    }});
    straightNormal(0.4, -0.2, 800);
    parallel::waitForAll({[]{
      pros::delay(400);
      mtrs->intake.controllerSet(1);
      mtrs->liftRaw.moveAbsolute(0, 100);
      pros::delay(600);
      mtrs->tilter.moveAbsolute(0, 100);
    }, []{
      turnNormal(0.8, 360, 1600);
      straightNormal(0.4, 0.4, 1000);
      straightNormal(0.4, -0.4, 1000);
    }});
    parallel::waitForAll({[]{
      pros::delay(800);
      mtrs->intake.moveRelative(-0.4, 100);
    }, []{
      turnNormal(1, 360 + 74, 2000);
    }});
    mtrs->tilter.moveAbsolute(1.2, 100);
    mtrs->liftRaw.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
    mtrs->liftRaw.moveAbsolute(-2.5, 80);
    straightNormal(0.5, 2.5, 2000);
    mtrs->intake.controllerSet(-0.4);
    pros::delay(1000);
    mtrs->intake.controllerSet(1);
    mtrs->liftRaw.moveAbsolute(0, 100);
    pros::delay(500);
    mtrs->tilter.moveAbsolute(0, 100);
    straightNormal(0.8, -0.25, 800);
    turnNormal(1.0, 360 + 22, 2000);
    straightNormal(0.8, 3, 3000);
    straightNormal(0.8, -1.2, 800);
    mtrs->intake.moveRelative(-0.4, 100);
    mtrs->tilter.moveAbsolute(1.2, 100);
    mtrs->liftRaw.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
    mtrs->liftRaw.moveAbsolute(-2.5, 80);
    turnNormal(0.8, 360 - 47, 2000); //utsukushii kimi ga kizu darake
    straightNormal(0.6, 0.55, 1400);
    mtrs->intake.controllerSet(-0.65);
    pros::delay(1000);
    //straightNormal(0.3, 0.3, 1000);
  } else if(which == "whip") {
    whipout();
  }
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  char buf[32];
  sprintf(buf, "Done @%lums", pros::millis() - startTime);
  printf("%s\n", buf);
  master.set_text(0, 0, buf);
  auto remainderTime = 15000 + (int)startTime - (int)pros::millis();
  //if(remainderTime > 0) pros::delay(remainderTime);
  printf("Time's up!\n");
  mtrs->all.moveVoltage(0);
  mtrs->intake.moveVoltage(0);
  mtrs->lift.moveVoltage(0);
  mtrs->tilter.moveAbsolute(0, 100);
  //pros::delay(4000);
}

void init_auto() {
  install_hawt_atom("auto", (void*)autonomous);
  install_hawt_atom("setBlue", (void*)setBlue);
  install_hawt_atom("grabAutonNames", (void*)grabAutonNames);
}
