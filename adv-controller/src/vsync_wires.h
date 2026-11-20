/* Vsync wires of a modded board (J18 pins 4 and 3): output and source vsync timed for the ESP, read over I2C
 * (i2c_vsync.c, boards with the J17 mod) (vsync_wires.c). Test channel: the TV5725 debug pin on J18 pin 5 (PB3). */
#ifndef VSYNC_WIRES_H
#define VSYNC_WIRES_H
#include "adv-link.h"

void VSW_Init(void);         /* after the clocks; harmless on a board without the wires (no edges, flags 0) */
void VSW_Task(void);        /* every main loop pass (well inside the cycle counter's 21.47 s wrap) */
void VSW_Get(AdvVsync *v);   /* the current reading (i2c_vsync.c frames it) */

/* Test channel (not sent to the ESP): the TV5725 debug pin (test bus, ESP GPIO12) wired to J18 pin 5 = PB3 */
typedef struct {
    uint32_t periodNs, minNs, maxNs;   /* mean / min / max of the last periods (0 = none current) */
    uint32_t edges, frames;            /* edges seen, frame marks */
    int32_t afterSrcNs, afterOutNs;    /* last mark after the LM1881 (PB5) / output (PB4) vsync; -1 = no reference */
    uint8_t alive;
} VswDebug;
void VSW_GetDebug(VswDebug *d);
void VSW_DebugStream(uint8_t port, uint8_t on);   /* one 'D <period> <after src> <after out>' line per frame */

#endif
