/* Vsync wires of a modded board (J18 pins 4 and 3): output and source vsync timed for the ESP, read over I2C
 * (i2c_vsync.c, boards with the J17 mod) (vsync_wires.c) */
#ifndef VSYNC_WIRES_H
#define VSYNC_WIRES_H
#include "adv-link.h"

void VSW_Init(void);         /* after the clocks; harmless on a board without the wires (no edges, flags 0) */
void VSW_Task(void);        /* every main loop pass (well inside the cycle counter's 21.47 s wrap) */
void VSW_Get(AdvVsync *v);   /* the current reading (i2c_vsync.c frames it) */

#endif
