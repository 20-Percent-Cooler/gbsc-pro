// Vsync wires over I2C, for boards with the J17 mod: the ADV Controller's HC32 answers on this I2C bus at
// ADV_VSYNC_I2C_ADDR and every read returns its latest vsync wire reading (adv-controller/src/i2c_vsync.c).
// Detected like the other chips (platformDetect's probe, then again every few seconds while absent: the HC32 can
// still be starting at the ESP's boot). Without the mod nothing answers and frame lock uses the debug pin.
// The vsync-pico add-on (a Pico on J15 timing the debug pin and the output vsync) answers at VSYNC_PICO_I2C_ADDR
// with the same frame and is used first. Loop context only (Wire).
#pragma once
#include <stdint.h>
#include "adv-link.h"

bool vsyncI2cPresent();                 // the HC32 answers (probes again every 5 s while it doesn't)
bool vsyncI2cRead(AdvVsync& v);         // one fresh frame; false when absent or the read was bad
struct VsyncI2cStats { uint32_t reads, fails, probes; bool present; };
VsyncI2cStats vsyncI2cStats();
