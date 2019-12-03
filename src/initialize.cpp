#include "main.h"
#include "tabu.hpp"
#include "entropy.hpp"
#include "followtest.hpp"
#include "sensors.hpp"
#include "pidtest.hpp"
#include "mtrs.hpp"
#include "display.hpp"
#include "superhot_compat.hpp"

void inputTask(void*) {
	while(true) {
		auto line = SuperHot::recv_line();
		tabu_handler(line);
	}
}

//These hawt functions exist to keep track of the locations of the things
//in hot memory at runtime. Using any hot symbols at compile-time will
//"melt" the cold section so to say, making it have to be reuploaded to keep
//hot references working and defying the point of it. 
std::unordered_map<std::string, void*> hawt_atoms;
void install_hawt_atom(const std::string& name, void* what) {
	hawt_atoms[name] = what;
}
void* retrieve_hawt_atom(const std::string& name) {
	return hawt_atoms[name];
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void r_initialize() {
	try {
		printf("こわいゆめだとしても rev5\n");
		init_random();
		pros::delay(200);
		//Causes memory permission error due to buggy LVGL multithreading.
		//Only when plugged into field.
		//init_display();
		init_sensors();
		mtrs = std::make_unique<Motors>();
		init_follow_test();
		init_pid_test();

		tabu_reply_on("ping", [](const Message& msg) -> json {
			return "Got ping message with content " + msg.content.to_string() + ".";
		});
		tabu_help("ping", {
			tstr("text"),
			treplyaction("say(it)")
		});

		pros::Task maInput(inputTask, nullptr, "tabu-input");
		SuperHot::registerTask(maInput);
	} catch(const char* initError) {
		printf("Got an init error: %s\n", initError);
		fflush(stdout);
		pros::delay(500);
		throw;
	} catch(const std::exception& except) {
		printf("Got an init error: %s\n", except.what());
		fflush(stdout);
		pros::delay(500);
		throw;
	} catch(...) {
		printf("Got some init error.\n");
		fflush(stdout);
		pros::delay(500);
		throw;
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
