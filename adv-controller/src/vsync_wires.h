/* Vsync wires of a modded board (J18 pins 4 and 3): output and source vsync timed for ADV_CMD_VSYNC (vsync_wires.c) */
#ifndef VSYNC_WIRES_H
#define VSYNC_WIRES_H
#include "adv-link.h"

void VSW_Init(void);         /* after the clocks; harmless on a board without the wires (no edges, flags 0) */
void VSW_Task(void);        /* every main loop pass (well inside the cycle counter's 21.47 s wrap) */
void VSW_Get(AdvVsync *v);   /* the reply to ADV_CMD_VSYNC */

#endif
