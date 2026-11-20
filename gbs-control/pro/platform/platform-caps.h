// Board capability mask: which optional chips this board has (pure, host-testable)
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../settings/settings-types.h"

enum : uint8_t { BOARD_AUTO = 0, BOARD_PRO = 1, BOARD_ORIGINAL = 2 };

struct ProbeResult { bool ack; uint8_t attempts; };
// advI2c: the ADV Controller's HC32 at 0x32 on this bus (the J17 mod): vsync wires over I2C (pro/link/vsync-i2c.h).
// Informational: it doesn't change the caps.
struct PlatformProbes { ProbeResult stv; ProbeResult pt; ProbeResult oled; ProbeResult advI2c; };

// Pro when the STV9426 or the PT2257 answers (two chips: one failed chip does not demote a Pro)
uint8_t platformCapsFrom(const PlatformProbes& p, uint8_t board);
// A Pro chip answered: GPIO1 drives the ADV Controller and GPIO2 is IR, whatever system.board says
bool platformProbesSayPro(const PlatformProbes& p);
// Detection rounds end when a Pro chip answered or after 5 rounds; the OLED alone does not end them (a slow
// STV9426 after a reset must not lose the race)
bool platformDetectDone(const PlatformProbes& p, uint8_t round);

uint8_t platformCaps();
void platformSetCaps(uint8_t caps);
inline bool hwHas(uint8_t cap) { return (platformCaps() & cap) == cap; }

// "adv", "osd", "audio", "ir", "oled", "encoder", "led", "original" → bit; 0 when unknown
uint8_t platformCapFromName(const char* name, size_t len);
