/* The vsync wires' readings for the ESP over I2C (boards with the J17 mod): read-only slave at ADV_VSYNC_I2C_ADDR
 * (i2c_vsync.c) */
#ifndef I2C_VSYNC_H
#define I2C_VSYNC_H
#include <stdint.h>

void I2CV_Init(void);              /* after VSW_Init */
void I2CV_Task(uint32_t nowMs);    /* every main loop pass: refreshes the frame every few ms */
uint32_t I2CV_Reads(void);         /* reads by the ESP since start */
uint8_t I2CV_Running(void);
uint32_t I2CV_Unsticks(void);       /* restarts after the bus was held stuck (a master reset mid-read) */

#endif
