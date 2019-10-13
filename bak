#include "main.h"
#include "okapi/api.hpp"
#include "blue_controller.hpp"
#include "mtrs.hpp"
#include "sensors.hpp"

//vex::Motor leftMotor1(0);
//vex::Motor rightMotor2(1);

double powered(int ctrl_power, double exp) {
	if(ctrl_power == 0) return 0;
	if(ctrl_power < 0) return -powered(-ctrl_power, exp);
	if(ctrl_power > 127) ctrl_power =  127;
	double ctrl = ctrl_power / 127.0;
	return std::pow(ctrl, exp);
}

volatile bool opcontrolActive = true;
volatile bool opcontrolActiveAck = true;
void pauseControl() {
	opcontrolActive = false;
	while(opcontrolActiveAck) pros::delay(1);
}

void resumeControl() {
	opcontrolActive = true;
	while(!opcontrolActiveAck) pros::delay(1);
}

void autonomous();

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

//split arcade base
//R1 - Toggle claw
//R2 - Toggle intake
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	mtrs->intake.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	uint32_t liftActivatedAt = pros::millis();
	while (true) {
		if(!opcontrolActive) {
			opcontrolActiveAck = false;
			pros::delay(10);
			continue;
		}
		double y_ctrl  = master.get_analog(ANALOG_LEFT_Y) / 127.0;
		double x_ctrl = master.get_analog(ANALOG_LEFT_X) / 127.0;
		mtrs->lift.controllerSet(-master.get_analog(ANALOG_RIGHT_Y) / 127.0);

		mtrs->left .moveVelocity(200 * (y_ctrl + x_ctrl));
		mtrs->right.moveVelocity(200 * (y_ctrl - x_ctrl));

		mtrs->tilter.moveVelocity(200 * (master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_L1)));
		mtrs->intake.moveVelocity(200 * (master.get_digital(DIGITAL_R2) - (master.get_digital(DIGITAL_L2) ? 0.75 : 0)));

		pros::delay(10);
	}
}
