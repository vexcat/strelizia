#include "atoms.hpp"
#include "main.h"

double miniTurnP = 0.007;
double miniTurnI = 0.00008;
double miniTurnD = 0.00003;
double testSpeed = 0.8;

void r_initialize();
void init_auto();

extern "C" {
	void                  vexDisplayPrintf( int32_t xpos, int32_t ypos, uint32_t bOpaque, const char *format, ... );
};

void initialize() {
  init_atoms();
  init_auto();
  install_hawt_atom("miniP", &miniTurnP);
  install_hawt_atom("miniI", &miniTurnI);
  install_hawt_atom("miniD", &miniTurnD);
  install_hawt_atom("testSpeed", &testSpeed);
  r_initialize();
}