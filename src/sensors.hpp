#pragma once
#include "main.h"

extern std::unique_ptr<pros::ADIPotentiometer> potPtr;
extern std::unique_ptr<pros::Imu> imuPtr;
extern std::unique_ptr<pros::ADIDigitalIn> bumper;
extern std::unique_ptr<pros::ADIDigitalIn> trayBumper;
void init_sensors();
