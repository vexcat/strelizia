#include "main.h"
#include "tabu.hpp"
#include "entropy.hpp"
#include "followtest.hpp"
#include "sensors.hpp"
#include "pidtest.hpp"
#include "mtrs.hpp"
#include "display.hpp"
#include "superhot_compat.hpp"
#include "blackbox.hpp"

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
std::unordered_map<std::string, void*>* hawt_atoms;
void install_hawt_atom(const std::string& name, void* what) {
	(*hawt_atoms)[name] = what;
}
void* retrieve_hawt_atom(const std::string& name) {
	return (*hawt_atoms)[name];
}

void init_atoms() {
	hawt_atoms = new std::unordered_map<std::string, void*>;
}

using bytes = std::vector<unsigned char>;

class CRC {
  uint32_t size;
  uint32_t table[256];
  public:
  CRC(uint32_t size, uint32_t poly);
  uint32_t operator()(const bytes& data, uint32_t accumulator = 0);
};

inline uint32_t mask32(int size) {
  return ((uint32_t)-1 >> (32 - size));
}

//From prosv5
CRC::CRC(uint32_t isize, uint32_t poly): size(isize) {
  for(uint32_t i = 0; i < 256; i++) {
    uint32_t acc = i << (size - 8);
    for(int j = 0; j < 8; j++) {
      if(acc & (1 << (size - 1))) {
        acc <<= 1;
        acc ^= poly;
      } else acc <<= 1;
    }
    table[i] = acc & mask32(size);
  }
}

uint32_t CRC::operator()(const bytes& data, uint32_t acc) {
  for(auto& d: data) {
    uint8_t i = (acc >> (size-8)) ^ d;
    acc = ((acc << 8) ^ table[i]) & mask32(size);
  }
  return acc;
}

CRC VEX_CRC32 = CRC(32, 0x04C11DB7);

bytes fromBuffer(void* buf, int len) {
	bytes vec;
	vec.resize(len);
	memcpy(vec.data(), buf, len);
	return vec;
}

bool willRunSelector = true;

extern "C" {
	void                  vexDisplayPrintf( int32_t xpos, int32_t ypos, uint32_t bOpaque, const char *format, ... );
};
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void r_initialize() {
	try {
		//Safety
		if(pros::competition::is_connected() && !pros::competition::is_disabled()) {
			willRunSelector = false;
		}
		if(willRunSelector) {
			init_display();
		}
		init_random();
		init_sensors();
		mtrs = std::make_unique<Motors>();
		init_follow_test();
		init_pid_test();
		init_blackbox();

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
		vexDisplayPrintf(10, 10, 1, "You broke it >.<\n");
		vexDisplayPrintf(10, 30, 1, "Got an init error: %s\n", initError);
		while(true) {
			pros::delay(500);
		}
	} catch(const std::exception& except) {
		vexDisplayPrintf(10, 10, 1, "You broke it >.<\n");
		vexDisplayPrintf(10, 30, 1, "Got an init error: %s\n", except.what());
		while(true) {
			pros::delay(500);
		}
	} catch(...) {
		vexDisplayPrintf(10, 10, 1, "You broke it >.<\n");
		while(true) {
			pros::delay(500);
		}
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
