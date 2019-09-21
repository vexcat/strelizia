#include "main.h"
#include "okapi/api.hpp"
#include "mtrs.hpp"

void doPID(double revs, int time, bool shouldTurn, okapi::IterativePosPIDController ctrl, okapi::AbstractMotor& out) {
  //Drive forward 20 revolutions.
  out.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
  out.tarePosition();
  ctrl.setTarget(revs);
  auto startTime = pros::millis();
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

bool amBlue = true;

void autonomous() {
  double startTime = pros::millis();
  straightPID(-2.5, 1500);
  turnPID(amBlue ? -1.5 : 1.5, 1200);
  mtrs->all.moveVelocity(0);
  mtrs->lift.moveVelocity(-200);
  pros::delay(1800);
  mtrs->lift.moveVelocity(0);
  pros::delay(700);
  mtrs->lift.moveVelocity(200);
  pros::delay(1600);
  mtrs->lift.moveVelocity(0);
  turnPID(amBlue ? -1.46 : 1.46, 1200);
  /*
  straightPID(-3.8, 2000);
   */
  mtrs->all.moveVelocity(-200);
  pros::delay(1200);
  amBlue ? mtrs->left.moveVelocity(50) : mtrs->right.moveVelocity(50);
  pros::delay(800);
  mtrs->turn.moveVelocity(amBlue ? 200 : -200);
  pros::delay(3000);
  mtrs->all.moveVelocity(0);
  double endTime = pros::millis();
  printf("Auton finished in %fs.\n", (endTime - startTime) / 1000.0);
}
