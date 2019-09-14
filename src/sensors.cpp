#include "main.h"
#include "okapi/api.hpp"
#include "tabu.hpp"

std::unique_ptr<pros::ADIUltrasonic> sonic;
std::unique_ptr<pros::ADIPotentiometer> pot;
std::unique_ptr<okapi::ADIEncoder> lenc;
std::unique_ptr<okapi::ADIEncoder> renc;
int32_t sonic_dist() {
  return sonic->get_value();
}
int32_t claw_pos() {
  return pot->get_value();
}
int32_t lenc_pos() {
  return lenc->get();
}
int32_t renc_pos() {
  return renc->get();
}
okapi::ADIEncoder& getLEnc() {
  return *lenc;
}
okapi::ADIEncoder& getREnc() {
  return *renc;
}

void init_sensors() {
  //sonic = std::unique_ptr<pros::ADIUltrasonic>(new pros::ADIUltrasonic('A', 'B'));
  pot = std::make_unique<pros::ADIPotentiometer>('H');
  //r: g, f
  //l: e, d
  lenc = std::make_unique<okapi::ADIEncoder>('E', 'F');
  renc = std::make_unique<okapi::ADIEncoder>('C', 'D');
  tabu_reply_on("sonic", [&]() -> json {
    return sonic_dist();
  });
  tabu_help("sonic", { tlabel("Read the sonic sensor"), treplyaction("say('Sonic: ' + it)") });
  tabu_reply_on("claw", [&]() -> json {
    return claw_pos();
  });
  tabu_help("claw", { tlabel("Read the claw's potentiometer sensor"), treplyaction("say('Claw: ' + it)") });
  tabu_reply_on("enc", [&]() -> json {
    return {lenc_pos(), renc_pos()};
  });
  tabu_help("enc", { tlabel("Read the encoders"), treplyaction("say('Encoders: ' + it[0] + ', ' + it[1])") });
  pros::delay(1000);
}
