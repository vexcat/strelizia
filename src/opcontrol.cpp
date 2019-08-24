#include "main.h"
#include "okapi/api.hpp"
#include "blue_controller.hpp"

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
	okapi::MotorGroup left   { 1,  2};
	okapi::MotorGroup right  {-3, -4};
	okapi::MotorGroup intake { 5};
	okapi::MotorGroup mgl    { 6};
	okapi::MotorGroup lift   { 7};
	okapi::MotorGroup claw   { 8};
	bool clawActive;
	bool intakeActive;
	intake.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	while (true) {
		if(!opcontrolActive) {
			opcontrolActiveAck = false;
			pros::delay(10);
			continue;
		}
		opcontrolActiveAck = true;
		int ctrl_y = master.get_analog(ANALOG_LEFT_Y);
		int ctrl_x = master.get_analog(ANALOG_LEFT_X);
		int ctrl_lift = master.get_analog(ANALOG_RIGHT_Y);
		lift.moveVoltage(12000 * powered(ctrl_lift, 1));
		if(master.get_digital(DIGITAL_UP)) {
			left.moveVelocity(200);
			right.moveVelocity(200);
		} else if(master.get_digital(DIGITAL_DOWN)) {
			left.moveVelocity(-200);
			right.moveVelocity(-200);
		} else {
			left .moveVoltage(12000 * powered(ctrl_y + ctrl_x, 1.4));
			right.moveVoltage(12000 * powered(ctrl_y - ctrl_x, 1.4));
		}
		mgl.controllerSet(master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));
		claw.controllerSet(clawActive);
		intake.controllerSet(intakeActive);
		pros::delay(10);
	}
}
