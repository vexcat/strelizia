#include "main.h"
#include "okapi/api.hpp"
#include "mtrs.hpp"

void returnToWall() {
  //Get to the wall
  mtrs->all.moveVelocity(-200);
  pros::delay(400);
  while((mtrs->all.getActualVelocity() / (int)mtrs->all.getGearing()) < -0.3) {
    pros::delay(10);
  }
  pros::delay(80);
  //Come back from the wall
  mtrs->all.controllerSet(0.5);
  pros::delay(400);
  mtrs->all.controllerSet(0);
  pros::delay(400);
}