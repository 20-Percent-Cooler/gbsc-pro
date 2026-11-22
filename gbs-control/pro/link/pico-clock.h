// The clock generator glided by the vsync-pico add-on (>= 0.7.0) instead of the ESP: the ESP sends the command,
// hands the I2C bus over and waits (WiFi keeps running, no I2C of its own) until the add-on gives it back. The
// handover signal is the add-on's debug-pin relay line on DEBUG_IN_PIN: held high > 3 ms = the add-on has the bus,
// then low > 3 ms = done (the relayed debug pin is low ~96 % of the time, high pulses < 1 ms).
#pragma once
#include <stdint.h>

// true: the add-on glided from -> to (the clock generator is at `to`); false: nothing happened, glide yourself
bool picoClockGlide(uint32_t fromHz, uint32_t toHz, uint16_t stepUs);
struct PicoClockStats { uint32_t glides, refused, sendFail, timeouts, lastMs; };
PicoClockStats picoClockStats();
