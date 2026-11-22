#ifdef ARDUINO
#include <Arduino.h>
#include "pico-clock.h"
#include "vsync-i2c.h"
#include "../log/log.h"

#ifndef DEBUG_IN_PIN
#define DEBUG_IN_PIN D6
#endif

static PicoClockStats gStats;
static uint8_t gSeq;

static uint8_t crc8(const uint8_t* p, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
    return c;
}

// si5351mcu's output divider and R for a frequency: a glide only rewrites the PLL while both stay the same
static void siDivider(uint32_t f, uint32_t& od, uint32_t& R) {
    R = 1; od = 900000000UL / f;
    while (od > 900) { R *= 2; od /= 2; }
    if (od % 2) od--;
}

// the line at one level for > ms (micros), within the deadline (millis)
static bool waitLevel(bool high, uint32_t forUs, uint32_t deadlineMs) {
    uint32_t since = 0;
    while ((int32_t)(millis() - deadlineMs) < 0) {
        if ((digitalRead(DEBUG_IN_PIN) != 0) == high) {
            if (!since) since = micros() | 1;
            if (micros() - since > forUs) return true;
        } else since = 0;
        yield();   // WiFi; no I2C here: the bus may be the add-on's
    }
    return false;
}

bool picoClockGlide(uint32_t fromHz, uint32_t toHz, uint16_t stepUs) {
    if (!vsyncI2cPicoCtrl() || fromHz == toHz) return false;
    uint32_t od0, r0, od1, r1;
    siDivider(fromHz, od0, r0); siDivider(toHz, od1, r1);
    if (od0 != od1 || r0 != r1) return false;
    uint8_t f[16] = {0xA5, ++gSeq, 0x10};
    memcpy(&f[3], &fromHz, 4); memcpy(&f[7], &toHz, 4); memcpy(&f[11], &stepUs, 2);
    f[13] = 10;   // 1 kHz steps, as the ESP's glide
    f[14] = 16; f[15] = crc8(f, 15);
    if (!vsyncI2cSend(f, 16)) { gStats.sendFail++; return false; }
    const uint32_t t0 = millis();
    if (!waitLevel(true, 3000, t0 + 60)) { gStats.refused++; return false; }   // not taken: the bus is still ours
    const uint32_t steps = (fromHz > toHz ? fromHz - toHz : toHz - fromHz) / 1000 + 1;
    const uint32_t limit = t0 + steps * ((uint32_t)stepUs + 600) / 1000 + 500;
    if (!waitLevel(false, 3000, limit)) {
        // still high: the add-on may still drive the bus. Wait for it longer rather than collide on the I2C bus
        gStats.timeouts++;
        if (!waitLevel(false, 3000, millis() + 3000))
            logPrintf(LC_SYNC, LL_INFO, PSTR("pico clock: the add-on kept the bus > %lu ms"), (unsigned long)(millis() - t0));
    }
    gStats.lastMs = millis() - t0;
    gStats.glides++;
    return true;
}

PicoClockStats picoClockStats() { return gStats; }
#endif
