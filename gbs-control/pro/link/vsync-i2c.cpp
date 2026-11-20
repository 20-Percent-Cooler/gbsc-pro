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
static const uint32_t kProbeEveryMs = 5000;
static const uint8_t kLostAfter = 10;   // bad reads in a row: probe again (a reset HC32, a loose wire)

static bool probeNow() {
    gProbes++;
    Wire.beginTransmission(ADV_VSYNC_I2C_ADDR);
    return Wire.endTransmission() == 0;
}

bool vsyncI2cPresent() {
    if (!hwHas(CAP_ADV)) return false;   // a standard board: no ADV Controller
    const uint32_t now = millis();
    if (!gInit) {
        gInit = true;
        gPresent = platformProbes().advI2c.ack;
        gProbeAt = now;
        if (gPresent) logPrintf(LC_SYNC, LL_INFO, PSTR("vsync wires: ADV Controller on I2C (0x%02X)"), ADV_VSYNC_I2C_ADDR);
    }
    if (!gPresent && now - gProbeAt >= kProbeEveryMs) {
        gProbeAt = now;
        if (probeNow()) {
            gPresent = true;
            gFailRun = 0;
            logPrintf(LC_SYNC, LL_INFO, PSTR("vsync wires: ADV Controller on I2C (0x%02X)"), ADV_VSYNC_I2C_ADDR);
        }
    }
    return gPresent;
}

bool vsyncI2cRead(AdvVsync& v) {
    if (!vsyncI2cPresent()) return false;
    uint8_t f[ADV_VSYNC_FRAME_BYTES];
    size_t n = Wire.requestFrom((uint8_t)ADV_VSYNC_I2C_ADDR, (uint8_t)sizeof(f));
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
