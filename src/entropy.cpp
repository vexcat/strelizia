#include "main.h"

//(Content of this file mainly comes from Elliot2/src/state.cpp)

//From https://crypto.stackexchange.com/questions/16219/cryptographic-hash-function-for-32-bit-length-input-keys
//Randomly permutes a 32bit value.
uint32_t perm32(uint32_t x) {
    int n = 12;
    do // repeat for n from 12 downto 1
        x = ((x>>8)^x)*0x6B+n;
    while( --n!=0 );
    return x;
}

//From https://stackoverflow.com/questions/2351087/what-is-the-best-32bit-hash-function-for-short-strings-tag-names
//Hashes a string to a 4-byte integer. Expect collisions
//past 2^16 strings.
unsigned int quick_hash(const char *str) {
   unsigned int h;
   unsigned const char *p;

   h = 0;
   for (p = (unsigned const char*)str; *p != '\0'; p++)
      h = 37 * h + *p;
   return h;
}

unsigned int gseed = 0;

//Initializes the global seed.
void init_random() {
    uint32_t seed = 0;
    //Get seed data from the environment
    for(int j = 0; j < 25; j++) {
        for(int i = 1; i < 8; i++) {
            seed ^= pros::ADIAnalogIn('A' + i).get_value() << ((i - 1) * 4);
        }
        seed = perm32(seed);
        seed += pros::battery::get_capacity() * 100;
        seed = perm32(seed);
        seed += pros::battery::get_current();
        seed = perm32(seed);
        seed += pros::battery::get_temperature();
        seed = perm32(seed);
        seed += pros::battery::get_voltage();
        seed = perm32(seed);
        seed += pros::c::controller_get_battery_level(pros::E_CONTROLLER_MASTER);
        seed = perm32(seed);
        seed += pros::c::controller_get_battery_capacity(pros::E_CONTROLLER_MASTER);
        seed = perm32(seed);

        pros::delay(10);
    }
    //XOR with current time
    seed ^= pros::millis();
    gseed = seed;
}

pros::Mutex randMutex;
//Gets a random 32bit value. Is thread-safe.
int get_random() {
    randMutex.take(TIMEOUT_MAX);
    auto ret = rand_r(&gseed);
    randMutex.give();
    return ret;
}
