/**
 * @file  i2c_probe.h
 * @brief Passive probe on the ESP's I2C bus (J17: PA4 = SCL, PA5 = SDA).
 *
 * Ported from GBSC-Pro-I2C. The HC32's I2C2 slave (i2c_vsync.c) only receives transfers addressed to it (0x32). The probe
 * watches the two pins directly: an external interrupt on SCL's rising edge samples a
 * bit, edges on SDA while SCL is high are START and STOP. It never drives the bus.
 *
 * Output goes to the ADV module's USB console port (I2CProbe_SetPort). Reads, register-pointer
 * writes (1 byte, before a read), address-only probes and the controller's own 0x32
 * traffic are only counted. Off at start-up. CLI:
 *   p1  register changes: the probe keeps copies of the TV5725 (all segments), Si5351 and
 *       STV9426 control registers and prints "C <ms> <chip> <reg> <old>><new>" only when a
 *       value changes (old ?? = not seen since p1 / pc). OLED data and OSD screen RAM are
 *       counted (po1: printed whole as "W" lines, to rebuild the screen on the host).
 *       Other chips (PT2257, unknown) print raw writes.
 *   p2  raw writes: "W <ms> <addr> <bytes...>" for every write of 2+ bytes
 *   p0  off      ps  statistics      pc  forget the register copies
 *   po1 / po0  print the OLED's transfers (SSD1306 commands and pixel data) / count only
 *   ph  read histogram: the TV5725 registers read most often, and reads per chip
 * Any NACKed or garbled transfer prints as "N <ms> <addr> ..." (!A / !D = NACK on the
 * address / data, ?partial = lost edges, +n = n bytes not stored).
 */
#ifndef I2C_PROBE_H
#define I2C_PROBE_H

#include <stdint.h>
#include "console.h"

void I2CProbe_Init(void);              /* after I2CV_Init(); leaves the probe off */
void I2CProbe_SetPort(const ConsolePort *usb);   /* the USB console: probe output goes there only */
void I2CProbe_ReportStuck(uint32_t stuckMs, uint8_t clocks, uint8_t released);   /* i2c_vsync.c bus recovery */
#define PROBE_MODE_OFF      (0U)
#define PROBE_MODE_CHANGES  (1U)
#define PROBE_MODE_RAW      (2U)

void I2CProbe_Enable(uint8_t mode);    /* PROBE_MODE_* */
void I2CProbe_Forget(void);            /* register copies back to "not seen" */
void I2CProbe_OledLog(uint8_t on);     /* po1 / po0: OLED transfers printed / only counted */
void I2CProbe_Task(void);              /* main loop: prints captured writes */
void I2CProbe_PrintStats(void);
void I2CProbe_SetBlind(uint8_t blind); /* around the HC32's own flash erase/program (nests) */
void I2CProbe_PrintHistogram(void);  /* "ph": the most polled registers since p1/p2 */

#endif /* I2C_PROBE_H */
