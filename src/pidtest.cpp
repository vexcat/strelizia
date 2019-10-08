#include "main.h"
#include "okapi/api.hpp"
#include "tabu.hpp"
#include "opcontrol.hpp"
#include "mtrs.hpp"
#include "automation_util.hpp"

struct PIDDataPoint {
  double time;
  double error;
  double p;
  double i;
  double d;
  double step;
};

class TestingController: public okapi::IterativePosPIDController {
  public:
  TestingController(
  double ikP,
  double ikI,
  double ikD,
  double ikBias,
  const okapi::TimeUtil &itimeUtil,
  std::unique_ptr<okapi::Filter> iderivativeFilter = std::make_unique<okapi::PassthroughFilter>()):
  okapi::IterativePosPIDController(ikP, ikI, ikD, ikBias, itimeUtil, std::move(iderivativeFilter)) {}
  double getProportionalFactor() {
    return kP * error;
  }
  double getIntegralFactor() {
    return integral;
  }
  double getDerivativeFactor() {
    return -kD * derivative;
  }
};

void init_pid_test() {
  tabu_reply_on("pid_test", [&](const Message& msg) -> json {
    puts(msg.content.dump().c_str());
    printf("hi\n");
    pauseControl();
    std::vector<PIDDataPoint> collectedData;
    //Create PID controller from parameters in msg.
    auto controller = TestingController(
      msg.number("kP"),
      msg.number("kI"),
      msg.number("kD"),
      msg.number("kBias"),
      okapi::TimeUtilFactory::create(),
      std::make_unique<okapi::AverageFilter<5>>()
    );
    controller.setOutputLimits(msg.number("max"), -msg.number("max"));
    auto& out = msg.boolean("turn") ? mtrs->turn : mtrs->all;
    //Drive forward 20 revolutions.
    out.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
    out.tarePosition();
    controller.setTarget(msg.number("revs"));
    auto startTime = pros::millis();
    auto endTime = pros::millis() + msg.integer("ms");
    bool useVoltage = msg.boolean("useVoltage");
    while(pros::millis() < endTime) {
      double step = controller.step(out.getPosition());
      collectedData.push_back({
        (double)(pros::millis() - startTime),
        controller.getError(),
        controller.getProportionalFactor(),
        controller.getIntegralFactor(),
        controller.getDerivativeFactor(),
        step
      });
      if(useVoltage) {
        out.moveVoltage(step * 12000);
      } else {
        out.moveVelocity(step * (int)out.getGearing());
      }
      pros::delay(10);
    }
    if(!msg.boolean("turn")) {
      returnToWall();
    }
    resumeControl();
    auto jarr = json::array();
    for(auto& entry: collectedData) {
      jarr.push_back({
        {"time", entry.time},
        {"error", entry.error},
        {"p", entry.p},
        {"i", entry.i},
        {"d", entry.d},
        {"step", entry.step}
      });
    }
    return {
      {"graphable", jarr}
    };
  });
  tabu_help("pid_test", {
    tlabel("Do a PID test"),
    tnum("kP"), tnum("kI"), tnum("kD"), tnum("kBias"),
    tnum("max"),
    tnum("ms"),
    tbool("useVoltage"),
    tbool("turn"),
    tnum("revs"),
    treplyaction("graph(it.graphable)")
  });
}