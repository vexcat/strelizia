#include "main.h"
#include "okapi/api.hpp"
#include "mtrs.hpp"

void doPID(double revs, int time, bool shouldTurn, okapi::IterativePosPIDController ctrl, okapi::AbstractMotor& out) {
  //Drive forward 20 revolutions.
  out.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  out.tarePosition();
  ctrl.setTarget(revs);
  auto endTime = pros::millis() + time;
  while(pros::millis() < endTime) {
    double step = ctrl.step(out.getPosition());
    out.moveVelocity(step * (int)out.getGearing());
    pros::delay(10);
  }
  printf("%f fV, %f fE on straight of %f\n", out.getActualVelocity(), ctrl.getError(), revs);
}

void straightPID(double revs, int time) {
  doPID(revs, time, false, okapi::IterativePosPIDController(
    1.0,
    0.007,
    0.009,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), mtrs->all);
}

void turnPID(double revs, int time) {
  //Create PID controller from parameters in msg.
  doPID(revs, time, true, okapi::IterativePosPIDController(
    1.0,
    0.007,
    0.024,
    0,
    okapi::TimeUtilFactory::create(),
    std::make_unique<okapi::AverageFilter<5>>()
  ), mtrs->turn);
}

bool amBlue = false;

void autonomous() {
  
}
