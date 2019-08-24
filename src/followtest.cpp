#include "main.h"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "scurve.hpp"
#include "tabu.hpp"
#include "opcontrol.hpp"
#include "ultrasonic.hpp"
  
enum TailKind {
  BY_TIME,
  BY_VEL,
  BY_DIST
};

struct Tail {
  TailKind kind;
  double loc;
  bool satisfiedBy(
    double elapsedTime,
    double vel,
    double pos
  ) {
    switch(kind) {
      case BY_TIME: return elapsedTime > loc;
      case BY_VEL : return vel > loc;
      case BY_DIST: return pos > loc;
    }
    return false;
  }
};

struct Trial {
  double d;
  double v;
  double a;
  double j;
  double kV;
  double kA;
  Tail beginTail;
  Tail endTail;
  bool stopOnFinish;
  okapi::AbstractMotor::brakeMode stopBrakeMode;
  bool feedbackOn;
};

struct Position {
  double time;
  double disp;
  double cVel;
  double mVel;
  double dVel;
};

struct TrialResults {
  Trial trial;
  std::vector<Position> sampledPosition;
  double finalVelocity;
};

double wheelDia = 4;
double maxVel = (200/60.0) * (M_PI * wheelDia);

double velToInches(double rpm) {
  return (rpm/60) * M_PI * wheelDia;
}

double toInches(double revs) {
  return revs * M_PI * wheelDia;
}

class CurveDriver {
  public:
  TrialResults& res;
  Trial& trial;
  SCurve curve;
  std::vector<Position>& data;
  okapi::MotorGroup& output;
  uint64_t beginTime;
  double beginReading;
  double lastUsedTime = 0;
  bool stopOnFinish;
  okapi::AbstractMotor::brakeMode stopBrakeMode;
  CurveDriver(TrialResults& itrial, okapi::MotorGroup& ioutput):
  res(itrial), trial(res.trial), curve(trial.v, trial.a, trial.j, trial.d),
  data(res.sampledPosition), output(ioutput), stopOnFinish(trial.stopOnFinish), stopBrakeMode(trial.stopBrakeMode) {
    output.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
    beginTime = pros::millis();
    beginReading = output.getPosition();
  }

  double elapsed() {
    return (pros::millis() - beginTime)/1000.0;
  }

  double disp() {
    return toInches(output.getPosition() - beginReading);
  }

  double realVel() {
    return velToInches(output.getActualVelocity());
  }

  double velForDisp(double d) {
    return curve.calc(curve.calcTimeForPos(d));
  }

  void driveForTime(double t) {
    double vel = curve.calc(t);
    double acc = curve.calcAccForTime(t);
    double dVel = vel * trial.kV + acc * trial.kA;
    output.moveVelocity(dVel);
    data.push_back({elapsed(), disp(), vel, realVel(), velToInches(dVel)});
    lastUsedTime = t;
    if(disp() > trial.d && stopOnFinish) {
      res.finalVelocity = realVel();
      output.setBrakeMode(stopBrakeMode);
      output.moveVelocity(0);
      throw res.finalVelocity;
    }
  }

  double predictedTimeRemaining() {
    return curve.timingWidth() - lastUsedTime;
  }
};

TrialResults doTest(Trial t) {
  TrialResults res;
  res.trial = t;
  okapi::MotorGroup out = {1, 2, -3, -4};
  auto driver = CurveDriver(res, out);
  double lastCurveTime = 0;
  double lastRealTime = 0;
  try {
    //Follow based on time until the tail is completed.
    while(!t.beginTail.satisfiedBy(
      driver.elapsed(),
      driver.realVel(),
      driver.disp()
    ) || (t.beginTail.kind == BY_VEL && t.beginTail.loc > driver.velForDisp(driver.disp()))) {
      driver.driveForTime(driver.elapsed());
      pros::delay(10);
    }
    //Follow based on distance until the end tail is not satisfied.
    while(t.endTail.satisfiedBy(
      driver.predictedTimeRemaining(),
      driver.realVel(),
      t.d - driver.disp()
    )) {
      lastRealTime = driver.elapsed();
      if(t.feedbackOn) {
        lastCurveTime = driver.curve.calcTimeForPos(driver.disp());
        driver.driveForTime(lastCurveTime);
      } else {
        lastCurveTime = lastRealTime;
        driver.driveForTime(lastRealTime);
      }
      pros::delay(10);
    }
    //Follow based on time until tWidth is reached.
    while(lastCurveTime + driver.elapsed() - lastRealTime < driver.curve.timingWidth()) {
      driver.driveForTime(lastCurveTime + driver.elapsed() - lastRealTime);
      pros::delay(10);
    }
    driver.res.finalVelocity = out.getActualVelocity();
  } catch(const double& finishVel) {
    driver.res.finalVelocity = out.getActualVelocity();
    printf("Test finished early.\n");
  }
  out.moveVelocity(1);
  for(int i = 0; i < 10; i++) {
    driver.data.push_back({driver.elapsed(), driver.disp(), 0, driver.realVel(), 0});
    pros::delay(10);
  }
  return res;
}


TrialResults recordMotorMax() {
  uint64_t beginTime = pros::millis();
  okapi::MotorGroup out  = {1, 2, -3, -4};
  out.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  double startDisp = out.getPosition();
  out.moveVelocity(200);
  double v = 0;
  std::vector<Position> list;
  while((v = std::abs(out.getActualVelocity())) > 5 || beginTime + 500 > pros::millis()) {
    list.push_back({(pros::millis() - beginTime)/1000.0, toInches(out.getPosition() - startDisp), velToInches(v)});
    if(beginTime + 1000 < pros::millis()) {
      out.moveVelocity(0);
    }
    pros::delay(10);
  }
  TrialResults res;
  res.sampledPosition = list;
  return res;
}

void returnToWall(okapi::MotorGroup& mtr) {
  int32_t s = 0;
  mtr.controllerSet(-1);
  while((s = sonic_dist()) > 520 || s == 0) {
    pros::delay(10);
  }
  mtr.controllerSet(0);
}

void init_follow_test() {
  tabu_reply_on("simple_follower.max_test", [&]() -> json {
    okapi::MotorGroup out  = {1, 2, -3, -4};
    pauseControl();
    auto data = recordMotorMax();
    returnToWall(out);
    resumeControl();
    auto jarr = json::array();
    for(auto& entry: data.sampledPosition) {
      jarr.push_back({
        {"time", entry.time},
        {"disp", entry.disp},
        {"cVel", entry.cVel},
        {"mVel", entry.mVel},
        {"dVel", entry.dVel}
      });
    }
    return jarr;
  }, true);
  tabu_reply_on("simple_follower.test", [&](const Message& message) -> json {
    okapi::MotorGroup out  = {1, 2, -3, -4};
    pauseControl();
    auto data = doTest({
      message.number("pos"),
      message.number("vel"),
      message.number("acc"),
      message.number("jrk"),
      message.number("kV"),
      message.number("kA"),
      {BY_DIST, 1},
      {BY_VEL, velToInches(24)},
      message.boolean("stopOnFinish"),
      (okapi::AbstractMotor::brakeMode)message.integer("stopBrakeMode"),
      message.boolean("feedbackEnabled")
    });
    returnToWall(out);
    resumeControl();
    auto jarr = json::array();
    for(auto& entry: data.sampledPosition) {
      jarr.push_back({
        {"time", entry.time},
        {"disp", entry.disp},
        {"cVel", entry.cVel},
        {"mVel", entry.mVel},
        {"dVel", entry.dVel}
      });
    }
    return {
      {"graphable", jarr},
      {"finalVel", data.finalVelocity}
    };
  }, true);
}