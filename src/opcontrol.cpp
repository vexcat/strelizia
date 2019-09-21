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
	mtrs->claw.setGearing(okapi::AbstractMotor::gearset::red);
	bool clawActive = false;
	bool intakeActive = false;
	bool clawDirty = false;
	bool clawOpened = false;
	bool liftActiveDown = false;
	int clawOpenTarget = 2100;
	mtrs->intake.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	uint32_t liftActivatedAt = pros::millis();
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
		mtrs->lift.moveVoltage(-12000 * powered(ctrl_lift, 1));
		if(master.get_digital(DIGITAL_UP)) {
			mtrs->left.moveVelocity(200);
			mtrs->right.moveVelocity(200);
		} else if(master.get_digital(DIGITAL_DOWN)) {
			mtrs->left.moveVelocity(-200);
			mtrs->right.moveVelocity(-200);
		} else {
			mtrs->left .moveVoltage(12000 * powered(ctrl_y + ctrl_x, 1.4));
			mtrs->right.moveVoltage(12000 * powered(ctrl_y - ctrl_x, 1.4));
		}
		mtrs->mgl.controllerSet(master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));
		mtrs->claw.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		if(clawActive) {
			if(!clawOpened) {
				mtrs->claw.moveVelocity(200);
				clawOpened = claw_pos() < clawOpenTarget;
			} else {
				mtrs->claw.moveVelocity(0);
			}
			clawDirty = true;
		} else if (clawDirty){
			clawOpened = false;
			clawDirty = claw_pos() < 2600;
			if(ctrl_lift < 63) {
				mtrs->claw.moveVoltage(-7000);
			} else {
				mtrs->claw.moveVoltage(0);
			}
		} else {
			clawOpened = false;
			mtrs->claw.moveVelocity(0);
		}
		mtrs->intake.controllerSet(intakeActive);
		if(master.get_digital_new_press(DIGITAL_R1)) {
			clawActive = !clawActive;
			clawOpenTarget = 2100;
		}
		if(master.get_digital_new_press(DIGITAL_R2)) {
			clawActive = !clawActive;
			clawOpenTarget = 1600;
		}
		if(ctrl_lift > 63 && !liftActiveDown) {
			liftActivatedAt = pros::millis();
			liftActiveDown = true;
		} else if(ctrl_lift <= 63 && liftActiveDown) {
			liftActiveDown = false;
		}
		if(master.get_digital_new_press(DIGITAL_B)) {
			intakeActive = !intakeActive;
		}
		if(master.get_digital_new_press(DIGITAL_Y)) {
			printf("enc: %ld, %ld\n", lenc_pos(), renc_pos());
		}
		pros::delay(10);
	}
}
