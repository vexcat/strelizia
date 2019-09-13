#include "main.h"
#include "tabu.hpp"

std::unique_ptr<pros::ADIUltrasonic> sonic;
std::unique_ptr<pros::ADIPotentiometer> pot;
int32_t sonic_dist() {
  return sonic->get_value();
}
int32_t claw_pos() {
  return pot->get_value();
}

void init_sensors() {
  sonic = std::unique_ptr<pros::ADIUltrasonic>(new pros::ADIUltrasonic('C', 'D'));
  pot = std::unique_ptr<pros::ADIPotentiometer>(new pros::ADIPotentiometer('H'));
  tabu_reply_on("sonic", [&]() -> json {
    return sonic_dist();
  });
  tabu_help("sonic", { tlabel("Read the sonic sensor"), treplyaction("say('Sonic: ' + it)") });
  tabu_reply_on("claw", [&]() -> json {
    return claw_pos();
  });
  tabu_help("claw", { tlabel("Read the claw's potentiometer sensor"), treplyaction("say('Claw: ' + it)") });
}
