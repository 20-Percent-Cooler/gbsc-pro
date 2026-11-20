#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include "vsync-i2c.h"
#include "../platform/platform-caps.h"
#include "../platform/platform-device.h"
#include "../log/log.h"

static bool gInit = false, gPresent = false;
static uint32_t gProbeAt = 0, gReads = 0, gFails = 0, gProbes = 0;
static uint8_t gFailRun = 0;
static uint8_t gAddr = ADV_VSYNC_I2C_ADDR;   // the slave in use: the vsync-pico add-on first, else the ADV Controller
static const uint32_t kProbeEveryMs = 5000;
static const uint8_t kLostAfter = 10;   // bad reads in a row: probe again (a reset HC32, a loose wire)

static bool probeAddr(uint8_t a) {
    gProbes++;
    Wire.beginTransmission(a);
    return Wire.endTransmission() == 0;
}

// The vsync-pico add-on (0x33: the TV5725's own vsync, every input) before the ADV Controller (0x32, the J17 mod)
// (a standard board has no ADV Controller: only the add-on)
static bool probeNow() {
    if (probeAddr(VSYNC_PICO_I2C_ADDR)) { gAddr = VSYNC_PICO_I2C_ADDR; return true; }
    if (hwHas(CAP_ADV) && probeAddr(ADV_VSYNC_I2C_ADDR)) { gAddr = ADV_VSYNC_I2C_ADDR; return true; }
    return false;
}

static void logFound() {
    logPrintf(LC_SYNC, LL_INFO, gAddr == VSYNC_PICO_I2C_ADDR ? PSTR("vsync wires: vsync-pico add-on on I2C (0x%02X)")
                                                             : PSTR("vsync wires: ADV Controller on I2C (0x%02X)"), gAddr);
}

bool vsyncI2cPresent() {
    const uint32_t now = millis();
    if (!gInit) {
        gInit = true;
        gPresent = probeAddr(VSYNC_PICO_I2C_ADDR);
        if (gPresent) gAddr = VSYNC_PICO_I2C_ADDR;
        else if (hwHas(CAP_ADV)) { gPresent = platformProbes().advI2c.ack; gAddr = ADV_VSYNC_I2C_ADDR; }
        gProbeAt = now;
        if (gPresent) logFound();
    }
    if (!gPresent && now - gProbeAt >= kProbeEveryMs) {
        gProbeAt = now;
        if (probeNow()) {
            gPresent = true;
            gFailRun = 0;
            logFound();
        }
    }
    return gPresent;
}

bool vsyncI2cRead(AdvVsync& v) {
    if (!vsyncI2cPresent()) return false;
    uint8_t f[ADV_VSYNC_FRAME_BYTES];
    size_t n = Wire.requestFrom(gAddr, (uint8_t)sizeof(f));
    if (n > sizeof(f)) n = sizeof(f);
    for (size_t i = 0; i < n; i++) f[i] = (uint8_t)Wire.read();
    while (Wire.available()) Wire.read();
    if (advVsyncFrameUnpack(f, n, &v)) {
        gReads++;
        gFailRun = 0;
        return true;
    }
    gFails++;
    if (++gFailRun >= kLostAfter) {
        gPresent = false;
        gProbeAt = millis();
        logPrintf(LC_SYNC, LL_INFO, PSTR("vsync wires: no valid I2C reply %u times, probing again"), kLostAfter);
    }
    return false;
}

VsyncI2cStats vsyncI2cStats() { return {gReads, gFails, gProbes, gPresent}; }
#endif
