#include "main.h"

namespace SuperHot {
  #ifdef SUPERHOT_ENABLED
	pros::task_t registerTask(pros::task_t it);
  #else
  inline pros::task_t registerTask(pros::task_t it) { return it; }
  #endif
	void execImage();

	struct SerInputLine {
		enum { TO_DIRECT, TO_PATCH } aim = TO_DIRECT;
		std::string text;
	};

  #ifdef SUPERHOT_ENABLED
	std::string recv_line();
  #else
  inline std::string recv_line() {
    std::string acc = "";
    int c = 0;
    while((c = getchar()) != -1  && c != '\n') {
      acc += c;
    }
    return acc;
  }
  #endif
}