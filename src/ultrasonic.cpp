#include "main.h"
#include "tabu.hpp"

std::unique_ptr<pros::ADIUltrasonic> sonic;
int32_t sonic_dist() {
  return sonic->get_value();
}

void init_sonic() {
  sonic = std::unique_ptr<pros::ADIUltrasonic>(new pros::ADIUltrasonic('C', 'D'));
  tabu_reply_on("sonic", [&]() -> json {
    return sonic_dist();
  });
}
