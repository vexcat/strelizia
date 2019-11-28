#include "main.h"

namespace SuperHot {
	void registerTask(pros::task_t it);
	void execImage();

	struct SerInputLine {
		enum { TO_DIRECT, TO_PATCH } aim = TO_DIRECT;
		std::string text;
	};

  #ifdef SUPERHOT_ENABLED
	std::string recv_line();
  #else
  std::string recv_line() {
    std::string acc = "";
    int c = 0;
    while((c = getchar()) != -1  && c != '\n') {
      acc += c;
    }
    return acc;
  }
  #endif
}