#pragma once
#include "main.h"

//Gets a random value thread safely.
int get_random();
//Initializes the global random seed.
void init_random();
//Randomly permutes a 32bit number.
uint32_t perm32(uint32_t x);
