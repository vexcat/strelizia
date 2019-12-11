#include "main.h"
#include "okapi/api.hpp"
#include "tabu.hpp"
#include "mtrs.hpp"

//Sensor pointers
std::unique_ptr<pros::ADIPotentiometer> potPtr;

void make_reader(const std::string& named, std::function<double()> callable) {
  tabu_reply_on("enc_" + named, [=]() -> json {
    printf("%f\n", callable());
    return json(callable());
  });
  tabu_help("enc_" + named, json::array({ treplyaction("say(it)") }));
}

void init_sensors() {
  potPtr = std::make_unique<pros::ADIPotentiometer>('G');
  make_reader("base", [&]() -> double { return mtrs->all.getPosition(); });
  make_reader("turn", [&]() -> double { return mtrs->turn.getPosition(); });
  make_reader("tilter", [&]() -> double { return mtrs->tilter.getPosition(); });
  make_reader("lift", [&]() -> double { return mtrs->liftRaw.getPosition(); });
  make_reader("intake", [&]() -> double { return mtrs->intake.getPosition(); });
  make_reader("pot", [&]() -> double { return potPtr->get_value(); });
}
