#pragma once

#include "main.h"
#include "tabu.hpp"

namespace bros {
  //A controller that returns values received from tabu.
  //Can be overridden by pressing a key on the real controller
  //or by moving a joystick on the real controller past 16 units.
  struct Controller {
    //Axis 0: Left X
    //Axis 1: Left Y, Inverted
    //Axis 2: Right X
    //Axis 3: Right Y, Inverted
    double axes[4];
    //X, A, B, Y
    //L1, R1, L2, R2
    //back, start, lclick, rclick
    //up, down, left, right
    bool buttons[16];
    //Holds last value of each button, used to let get_digital_new_press work.
    //Indexed by a pros::controller_digital_e_t.
    bool lastButtons[32];
    std::string prefix;
    std::unique_ptr<pros::Controller> realControl;
    //Returns num, or 0 when its absolute value is less than 16.
    int dz(int num) {
      if(std::abs(num) < 16) return 0;
      return num;
    }
    //Gets joystick position from last tabu message or the real
    //controller.
    std::int32_t get_analog ( pros::controller_analog_e_t channel ) {
      int realRet;
      if(realControl && dz(realRet = realControl->get_analog(channel))) {
        return realRet;
      }
      if(channel == ANALOG_LEFT_X) return axes[0] * 127;
      if(channel == ANALOG_LEFT_Y) return -axes[1] * 127;
      if(channel == ANALOG_RIGHT_X) return axes[2] * 127;
      if(channel == ANALOG_RIGHT_Y) return -axes[3] * 127;
      return 0;
    }
    //Returns whether a button is pressed from last tabu messsage
    //or the real controller.
    std::int32_t get_digital ( pros::controller_digital_e_t button ) {
      int realRet;
      if(realControl && dz(realRet = realControl->get_digital(button))) {
        return realRet;
      }
      if(button == DIGITAL_X) return buttons[0];
      if(button == DIGITAL_A) return buttons[1];
      if(button == DIGITAL_B) return buttons[2];
      if(button == DIGITAL_Y) return buttons[3];
      if(button == DIGITAL_L1) return buttons[4];
      if(button == DIGITAL_R1) return buttons[5];
      if(button == DIGITAL_L2) return buttons[6];
      if(button == DIGITAL_R2) return buttons[7];
      if(button == DIGITAL_UP)    return buttons[12];
      if(button == DIGITAL_DOWN)  return buttons[13];
      if(button == DIGITAL_LEFT)  return buttons[14];
      if(button == DIGITAL_RIGHT) return buttons[15];
      return 0;
    }
    //Detects rising-edge button presses using get_digital() and lastButtons.
    std::int32_t get_digital_new_press(pros::controller_digital_e_t button) {
      int old = lastButtons[button];
      lastButtons[button] = get_digital(button);
      if(lastButtons[button] && !old) return true;
      return false;
    }
    //Starts listening for tabu mesesages.
    void init_listen() {
      tabu_on(prefix + ".move", [&](const Message& msg) {
        axes[msg.integer("axis")] = msg.number("value");
      });
      tabu_on(prefix + ".key", [&](const Message& msg) {
        buttons[msg.integer("num")] = msg.boolean("pressed");
      });
    }
    //Constructs a controller, and also sets a realControl for
    //overriding tabu control.
    Controller(pros::controller_id_e_t id): realControl(new pros::Controller(id)) {
      if(id == pros::E_CONTROLLER_MASTER) {
        prefix = "blue_control";
      } else {
        prefix = "blue_control_partner";
      }
      init_listen();
    }
    //Constructs a controller listening with a given prefix for tabu control messages.
    Controller(std::string prefix): prefix(prefix) {
      init_listen();
    }
  };
}