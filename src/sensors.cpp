#include "main.h"
#include "okapi/api.hpp"
#include "tabu.hpp"
#include "mtrs.hpp"

//No longer any extra sensors on robot.
void init_sensors() {
  tabu_reply_on("enc_base", []() -> json {
    return mtrs->all.getPosition();
  });
  tabu_help("enc_base", json::array({ treplyaction("say(it)") }));
  tabu_reply_on("tilter_enc", []() -> json {
    return mtrs->tilter.getPosition();
  });
  tabu_help("enc_tilter", json::array({ treplyaction("say(it)") }));
  tabu_reply_on("enc_lift", []() -> json {
    return mtrs->lift.getPosition();
  });
  tabu_help("enc_lift", json::array({ treplyaction("say(it)") }));
  tabu_reply_on("enc_intake", []() -> json {
    return mtrs->intake.getPosition();
  });
  tabu_help("enc_intake", json::array({ treplyaction("say(it)") }));
}
