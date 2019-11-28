#include "main.h"
#include "display/lvgl.h"
#include <vector>
#include <algorithm>
#include "display.hpp"
#include "atoms.hpp"

//---------------------------------------
//  Auton Selector
//---------------------------------------
lv_obj_t* autoSelectorObj;
lv_obj_t* buttonTemplate;
int selectedIndex = 2;
std::string currentlySelected;
std::string getSelectedAuton() {
  return currentlySelected;
}
std::vector<std::string> autonNames;
std::vector<std::pair<lv_obj_t*, lv_obj_t*>> selectors;

//Adds new autons to the screen
void addAuton(std::string byName) {
  if(std::find(autonNames.begin(), autonNames.end(), byName) != autonNames.end()) {
    throw "ur data is bad and u should feel bad";
  } else {
    autonNames.push_back(byName);
    if(selectors.size() < 8) {
      int i = selectors.size();
      lv_obj_t* button = lv_btn_create(autoSelectorObj, buttonTemplate);
      lv_obj_set_hidden(button, false);
      int bx = i%4;
      int by = i/4;
      lv_obj_set_pos(button, bx * 95 + 90, by * 115 + 10);
      lv_obj_t* label = lv_label_create(button, NULL);
      lv_label_set_text(label, byName.c_str());
      lv_style_t* stylish = lv_obj_get_style(label);
      stylish->text.color = LV_COLOR_HEX(0x000000);
      lv_btn_set_action(button, LV_BTN_ACTION_CLICK, [](lv_obj_t* obj) -> lv_res_t {
        int j = 0;
        for(auto &pair: selectors) {
          if(pair.first == obj) {
            lv_btn_set_state(pair.first, LV_BTN_STATE_TGL_REL);
            currentlySelected = autonNames[j];
          } else {
            lv_btn_set_state(pair.first, LV_BTN_STATE_REL);
          }
          j++;
        }

        return LV_RES_OK;
      });
      selectors.push_back({button, label});
    }
  }
}

//Removes autons from the screen
void removeAuton(std::string byName) {
  auto loc = std::find(autonNames.begin(), autonNames.end(), byName);
  if(loc == autonNames.end()) {
    throw "ur data is bad and u should feel bad";
  } else {
    auto loc2 = selectors.begin() + (loc - autonNames.begin());
    lv_obj_del(loc2->first);
    selectors.erase(loc2);
    autonNames.erase(loc);
    for(int i = 0; i < selectors.size(); i++) {
      int bx = i%4;
      int by = i/4;
      lv_obj_set_pos(selectors[i].first, bx * 95 + 90, by * 115 + 10);
    }
  }
}

//Initializes the auton selector
void autoSelector() {
  ((void (*)(bool))retrieve_hawt_atom("setBlue"))(true);
  autoSelectorObj = lv_obj_create(lv_scr_act(), NULL);
  lv_obj_set_size(autoSelectorObj, 480, 240);
  lv_obj_set_style(autoSelectorObj, &lv_style_plain);
  buttonTemplate = lv_btn_create(autoSelectorObj, NULL);
  lv_obj_set_size(buttonTemplate, 85, 105);
  lv_obj_set_hidden(buttonTemplate, true);
  static lv_obj_t* colorButton = lv_btn_create(autoSelectorObj, NULL);
  lv_btn_set_toggle(colorButton, true);
  static lv_style_t redStyle;
  static lv_style_t blueStyle;
  static lv_style_t redDullStyle;
  static lv_style_t blueDullStyle;
  lv_style_copy(& redStyle, &lv_style_btn_rel);
  lv_style_copy(&blueStyle, &lv_style_btn_tgl_rel);
  lv_style_copy(& redDullStyle, &lv_style_btn_pr);
  lv_style_copy(&blueDullStyle, &lv_style_btn_tgl_pr);
    redStyle.body.grad_color = LV_COLOR_HEX(0xFF0000);
    redStyle.body.main_color = LV_COLOR_HEX(0xFF0000);
  blueStyle.body.grad_color = LV_COLOR_HEX(0x0000FF);
  blueStyle.body.main_color = LV_COLOR_HEX(0x0000FF);
    redDullStyle.body.grad_color = LV_COLOR_HEX(0x7F0000);
    redDullStyle.body.main_color = LV_COLOR_HEX(0x7F0000);
  blueDullStyle.body.grad_color = LV_COLOR_HEX(0x00007F);
  blueDullStyle.body.main_color = LV_COLOR_HEX(0x00007F);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_REL, &redStyle);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_PR, &redDullStyle);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_TGL_REL, &blueStyle);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_TGL_PR, &blueDullStyle);
  lv_obj_set_pos(colorButton, 10, 10);
  lv_obj_set_size(colorButton, 70, 220);
  static lv_obj_t* colorLabel = NULL;
  colorLabel = lv_label_create(colorButton, NULL);
  //TGL makes blue
  lv_btn_set_state(colorButton, LV_BTN_STATE_TGL_REL);
  lv_label_set_text(colorLabel, "BLUE");
  lv_btn_set_action(colorButton, LV_BTN_ACTION_CLICK, [](lv_obj_t* obj) -> lv_res_t {
    if(lv_btn_get_state(colorButton) == LV_BTN_STATE_REL || lv_btn_get_state(colorButton) == LV_BTN_STATE_PR) {
      ((void (*)(bool))retrieve_hawt_atom("setBlue"))(false);
      lv_label_set_text(colorLabel, "RED");
    } else {
      ((void (*)(bool))retrieve_hawt_atom("setBlue"))(true);
      lv_label_set_text(colorLabel, "BLUE");
    }
    
    return LV_RES_OK;
  });
  addAuton("#No Auton");
  //Load existing autons
  auto vec = ((std::vector<std::string> (*)())retrieve_hawt_atom("grabAutonNames"))();
  for(auto &elem: vec) {
    addAuton(elem);
  }
  lv_btn_set_state(selectors[selectedIndex].first, LV_BTN_STATE_TGL_REL);
  currentlySelected = vec[selectedIndex - 1];
}

//---------------------------------------
//  Team Logo
//---------------------------------------
extern "C" {
  extern const lv_img_dsc_t cougarImage;
}
lv_obj_t* image;
void putImage() {
  image = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(image, &cougarImage);
}

//-------------------------------------
//  UI Executor
//-------------------------------------
int lastCompStatus = -1;
//Responsible for running all V5 Brain Screen tasks.
void uiExecutor(void*) {
  pros::delay(200);
  putImage();
  autoSelector();
  while(true) {
    int currentStatus = pros::competition::get_status();
    if(currentStatus != lastCompStatus) {
      lastCompStatus = currentStatus;
      if(currentStatus & COMPETITION_CONNECTED) {
        if(currentStatus & COMPETITION_DISABLED) {
          lv_obj_set_hidden(autoSelectorObj, false);
        } else {
          lv_obj_set_hidden(autoSelectorObj, true);
        }
      } else {
        if(currentStatus & COMPETITION_AUTONOMOUS) {
          lv_obj_set_hidden(autoSelectorObj, true);
        } else {
          lv_obj_set_hidden(autoSelectorObj, false);
        }
      }
    }
    pros::delay(50);
  }
}

void init_display() {
  pros::Task(uiExecutor, NULL, "UI Executor");
}