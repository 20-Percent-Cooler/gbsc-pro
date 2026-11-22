#ifndef FRAMESYNC_H_
#define FRAMESYNC_H_

// fast digitalRead()
#if defined(ESP8266)
#define digitalRead(x) ((GPIO_REG_READ(GPIO_IN_ADDRESS) >> x) & 1)
#ifndef DEBUG_IN_PIN
#define DEBUG_IN_PIN D6
#endif
#else // Arduino
// fastest, but non portable (Uno pin 11 = PB3, Mega2560 pin 11 = PB5)
//#define digitalRead(x) bitRead(PINB, 3)
#include "fastpin.h"
#define digitalRead(x) fastRead<x>()
// no define for DEBUG_IN_PIN
#endif

#include <ESP8266WiFi.h>
#include "pro/input/input-types.h"
#include "pro/link/adv-link-device.h"
#include "pro/link/vsync-i2c.h"
#include "pro/link/pico-clock.h"

uint16_t profileSourceLines();                                          // gbs-control.ino
bool menuSourceInterlaced(uint8_t videoStandardInput, uint16_t vperiodIf);   // pro/ui/menu-signal.cpp

// FS_DEBUG:      full verbose debug over serial
// FS_DEBUG_LED:  just blink LED (off = adjust phase, on = normal phase)
//#define FS_DEBUG
//#define FS_DEBUG_LED
// #define FRAMESYNC_DEBUG

#ifdef FRAMESYNC_DEBUG
#define fsDebugPrintf(...) SerialM.printf(__VA_ARGS__)
#else
#define fsDebugPrintf(...)
#endif

// Two rising edges of the vsync on the debug pin. The ISR only stores cycle counts: it used to call
// detach/attachInterrupt (flash code) and stayed attached when a measurement timed out, so a vsync edge during
// a flash operation (web upload, backup, settings save: the cache is off) ran flash code from the ISR -
// Exception 0 crashes in whatever function was hit. stop() detaches from the loop context.
namespace MeasurePeriod {
    volatile uint32_t stopTime, startTime;
    volatile uint32_t armed;
    volatile uint8_t edges;

    void IRAM_ATTR _risingEdgeISR()
    {
        uint32_t t;
        __asm__ __volatile__("rsr %0,ccount" : "=a"(t));
        if (edges == 0) { startTime = t; armed = 1; edges = 1; }
        else if (edges == 1) { stopTime = t; edges = 2; }
    }

    void start() {
        startTime = 0;
        stopTime = 0;
        armed = 0;
        edges = 0;
        attachInterrupt(DEBUG_IN_PIN, _risingEdgeISR, RISING);
    }

    void stop() { detachInterrupt(DEBUG_IN_PIN); }   // always, also after a timeout
}

// output.clock_retune 3 (slow glide): the clock generator's actual output, moved 1 kHz per output frame towards
// rto->freqExtClockGen from loop() (clockGlideTick), ~6 ppm a frame: a monitor sees a slow drift, not a new mode.
// 0 when no glide runs.
static uint32_t gGlideHz = 0, gGlideAt = 0;

void clockGlideTick() {
    if (!gGlideHz) return;
    const uint32_t now = millis();
    if (now - gGlideAt < 16) return;
    gGlideAt = now;
    const uint32_t target = rto->freqExtClockGen;
    if (gGlideHz + 1000 < target) gGlideHz += 1000;
    else if (gGlideHz > target + 1000) gGlideHz -= 1000;
    else gGlideHz = target;
    Si.setFreq(0, gGlideHz);
    if (gGlideHz == target) gGlideHz = 0;
}

void setExternalClockGenFrequencySmooth(uint32_t freq) {
    uint32_t current = gGlideHz ? gGlideHz : rto->freqExtClockGen;   // a slow glide still running: from where it is

    rto->freqExtClockGen = freq;

    if (uopt->clockRetune == 1) { gGlideHz = 0; Si.setFreq(0, freq); return; }   // step: one write, one frame
    // glide by the vsync-pico add-on (>= 0.7.0): it takes the bus, glides, gives it back (pro/link/pico-clock.h)
    if (uopt->clockRetune == 0 && current != freq && (current > freq ? current - freq : freq - current) < 750000 &&
        picoClockGlide(current, freq, 240)) { gGlideHz = 0; return; }   // 240 us + ~250 us per 400 kHz write: ~150 ms, as the ESP
    if (uopt->clockRetune == 3 && current != freq) {                               // slow glide (clockGlideTick)
        const uint32_t d = current > freq ? current - freq : freq - current;
        if (d < 750000) { gGlideHz = current; gGlideAt = millis(); return; }
    }
    gGlideHz = 0;

    constexpr uint32_t STEP_SIZE_HZ = 1000;

    if (current > rto->freqExtClockGen) {
        if ((current - rto->freqExtClockGen) < 750000) {
            while (current > (rto->freqExtClockGen + STEP_SIZE_HZ)) {
                current -= STEP_SIZE_HZ;
                Si.setFreq(0, current);
                handleWiFi(0);
            }
        }
    } else if (current < rto->freqExtClockGen) {
        if ((rto->freqExtClockGen - current) < 750000) {
            while ((current + STEP_SIZE_HZ) < rto->freqExtClockGen) {
                current += STEP_SIZE_HZ;
                Si.setFreq(0, current);
                handleWiFi(0);
            }
        }
    }

    Si.setFreq(0, rto->freqExtClockGen);
}

template <class GBS, class Attrs>
class FrameSyncManager
{
private:
    typedef typename GBS::STATUS_VDS_VERT_COUNT VERT_COUNT;
    typedef typename GBS::VDS_HSYNC_RST HSYNC_RST;
    typedef typename GBS::VDS_VSYNC_RST VSYNC_RST;
    typedef typename GBS::VDS_VS_ST VSST;
    typedef typename GBS::template Tie<VSYNC_RST, VSST> VRST_SST;

    static const uint8_t debugInPin = Attrs::debugInPin;
    static const int16_t syncCorrection = Attrs::syncCorrection;
    static const int32_t syncTargetPhase = Attrs::syncTargetPhase;

    static bool syncLockReady;
    static float smoothedFpsInput;   // EMA of the input FPS for runFrequency(); 0 = unset
    static uint8_t delayLock;
    static int16_t syncLastCorrection;

    /// Set to -1 if uninitialized.
    /// Reset with syncLastCorrection.
    static float maybeFreqExt_per_videoFps;

    // Vsync wires (modded board: J18 pins 4 and 3, read over I2C with the J17 mod). The ADV Controller times the output vsync and the
    // LM1881's source vsync; the debug pin then only measures once per reset() and scan type, for the constant
    // between the wires' phase and the pin's (VS_OUT vs the VDS vblank, the LM1881 vs the IF vsync: a fixed delay per
    // mode). Progressive and interlaced keep their own: frame lock isn't reset on a 240p <-> 480i switch with the clock
    // generator, and the IF vsync sits ~0.25 ms further from the LM1881 in 480i (N64: 772 vs 1026 us).
    static bool wireCalNeeded[2];    // [interlaced] reset(): the next wire reading in that scan type also measures the pin
    static int32_t wireOffset[2];    // [interlaced] ESP cycles: pin phase = wire phase + wireOffset
    static const int32_t kNoPinOffsetUs = 1150;
    static bool lastWasWire;         // the last vsyncPeriodAndPhase() came from the wires
    static uint8_t wireFlags;        // ADV_VS_* of the last wire reading

    // The wires' periods and phase (ESP cycles), when a fresh, plausible reading exists. An LM1881 source (the ADV
    // Controller's wires) follows the RGB SCART / RGB SoG sync only: on the other inputs it sees another line. A
    // timer on the TV5725's own source vsync (the debug pin, ADV_VS_ANY_INPUT: the vsync-pico add-on) is valid on all.
    static bool wireSample(int32_t *periodInput, int32_t *periodOutput, int32_t *phase)
    {
        const bool rgb = uopt->activeInputType == InputTypeRGBs || uopt->activeInputType == InputTypeRGsB;
        AdvVsync v;
        if (!vsyncI2cRead(v)) return false;   // no J17 mod / add-on, or no valid reply: the debug pin
        if (!rgb && !(v.flags & ADV_VS_ANY_INPUT)) return false;
        wireFlags = v.flags;
        const uint8_t all = ADV_VS_SRC | ADV_VS_OUT | ADV_VS_PHASE;
        if ((v.flags & all) != all) return false;
        const uint32_t src = v.srcPeriodNs, out = v.outPeriodNs;
        if (src < 11600000UL || src > 25000000UL) return false;            // 40..86 Hz
        if ((out > src ? out - src : src - out) > src / 16) return false;   // within 6 %: one signal, one output
        const uint32_t mhz = ESP.getCpuFreqMHz();
        *periodInput = (int32_t)((uint64_t)src * mhz / 1000U);
        if (periodOutput) *periodOutput = (int32_t)((uint64_t)out * mhz / 1000U);
        *phase = (int32_t)((uint64_t)(v.phaseNs % src) * mhz / 1000U);
        return true;
    }

    static int32_t wrapPhase(int32_t ph, int32_t period)   // into [0, period)
    {
        ph %= period;
        return ph < 0 ? ph + period : ph;
    }

    // Longest wait for two debug-pin edges: two periods at 47 Hz (the lowest rate getSourceFieldRate() accepts)
    // = 42.6 ms, plus margin.
    static constexpr uint32_t SAMPLE_TIMEOUT_MS = 60;
    static uint32_t sampleTimeoutCycles()
    {
        return (uint32_t)ESP.getCpuFreqMHz() * 1000U * SAMPLE_TIMEOUT_MS;
    }

    // Sample vsync start and stop times from debug pin.
    static bool vsyncOutputSample(uint32_t *start, uint32_t *stop)
    {
        yield();
        ESP.wdtDisable();
        MeasurePeriod::start();

        // Two rising edges = one period. Bounded in time, not in loop passes: with no edges on the debug pin (a broken
        // or unwired DEBUG_IN_PIN) the old 3,000,000 passes kept the ESP away from WiFi for about a second per call.
        const uint32_t t0 = ESP.getCycleCount();
        const uint32_t limit = sampleTimeoutCycles();
        while (ESP.getCycleCount() - t0 < limit) {
            if (MeasurePeriod::armed) {
                MeasurePeriod::armed = 0;
                delay(7);
                WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
            }
            if (MeasurePeriod::stopTime > 0) {
                break;
            }
        }
        MeasurePeriod::stop();
        *start = MeasurePeriod::startTime;
        *stop = MeasurePeriod::stopTime;
        ESP.wdtEnable(0);
        WiFi.setSleepMode(WIFI_NONE_SLEEP);

        if ((*start >= *stop) || *stop == 0 || *start == 0) {
            // ESP.getCycleCount() overflow oder no pulse, just fail this round
            return false;
        }

        return true;
    }

    // Input and output vsync periods and their phase, in ESP cycles: from the vsync wires when they run (the debug
    // pin once per reset() for their offset), else from the debug pin
    static bool vsyncPeriodAndPhase(int32_t *periodInput, int32_t *periodOutput, int32_t *phase)
    {
        int32_t wIn, wOut, wPh;
        lastWasWire = false;
        if (!wireSample(&wIn, &wOut, &wPh)) return pinPeriodAndPhase(periodInput, periodOutput, phase);
        const uint8_t m = menuSourceInterlaced(rto->videoStandardInput, profileSourceLines()) ? 1 : 0;
        if (wireCalNeeded[m]) {
            int32_t pIn, pOut, pPh;
            if (!pinPeriodAndPhase(&pIn, &pOut, &pPh)) {
                // A standard board whose ESP doesn't see the debug pin (it reaches J15 only), timed by the vsync-pico
                // add-on: no calibration possible. Estimate: the pin's input mark (IF vsync, bus 0) comes ~1150 us
                // after the add-on's source mark (the sync processor's vsync), N64 240p on a PRO, 2026-11-20.
                if (!(wireFlags & ADV_VS_ANY_INPUT)) return false;
                const uint32_t mhz = ESP.getCpuFreqMHz();
                wireOffset[m] = -kNoPinOffsetUs * (int32_t)mhz;
                wireCalNeeded[m] = false;
                logPrintf(LC_SYNC, LL_INFO, PSTR("frame lock: vsync-pico, %s, no debug pin at the ESP: offset estimated %ld us"),
                          m ? "interlaced" : "progressive", (long)(wireOffset[m] / (int32_t)mhz));
                if (periodInput) *periodInput = wIn;
                if (periodOutput) *periodOutput = wOut;
                if (phase) *phase = wrapPhase(wPh + wireOffset[m], wIn);
                lastWasWire = true;
                return true;
            }
            int32_t d = wrapPhase(pPh - wPh, wIn);
            wireOffset[m] = d > wIn / 2 ? d - wIn : d;   // the short way round
            wireCalNeeded[m] = false;
            const uint32_t mhz = ESP.getCpuFreqMHz();
            logPrintf(LC_SYNC, LL_INFO, PSTR("frame lock: vsync wires, %s phase offset %ld us (pin %ld us, wires %ld us)"),
                      m ? "interlaced" : "progressive", (long)(wireOffset[m] / (int32_t)mhz),
                      (long)(pPh / (int32_t)mhz), (long)(wPh / (int32_t)mhz));
            if (periodInput) *periodInput = pIn;
            if (periodOutput) *periodOutput = pOut;
            if (phase) *phase = pPh;
            return true;
        }
        if (periodInput) *periodInput = wIn;
        if (periodOutput) *periodOutput = wOut;
        if (phase) *phase = wrapPhase(wPh + wireOffset[m], wIn);
        lastWasWire = true;
        return true;
    }

    // Sample input and output vsync periods and their phase
    // difference in microseconds
    // The debug pin is switched here and only here for frame lock (bus 0 = IF vertical period for the input,
    // 2 = VDS vblank for the output) and given back as found: with the vsync wires running, the pin keeps the
    // caller's signal except for the one calibration per reset()
    static bool pinPeriodAndPhase(int32_t *periodInput, int32_t *periodOutput, int32_t *phase)
    {
        const uint8_t busSel = GBS::TEST_BUS_SEL::read();
        fsDebugPrintf("pinPeriodAndPhase(), TEST_BUS_SEL=%d\n", busSel);

        uint32_t inStart, inStop, outStart, outStop;
        uint32_t inPeriod, outPeriod, diff;

        if (busSel != 0x0) GBS::TEST_BUS_SEL::write(0x0);
        if (!vsyncInputSample(&inStart, &inStop)) {
            GBS::TEST_BUS_SEL::write(busSel);
            return false;
        }

        GBS::TEST_BUS_SEL::write(0x2); // 0x2 = VDS (t3t50t4) // measure VDS vblank (VB ST/SP)
        inPeriod = (inStop - inStart); //>> 1;
        const bool outOk = vsyncOutputSample(&outStart, &outStop);
        GBS::TEST_BUS_SEL::write(busSel);
        if (!outOk) {
            return false;
        }
        outPeriod = (outStop - outStart); //>> 1;


        diff = (outStart - inStart) % inPeriod;
        if (periodInput)
            *periodInput = inPeriod;
        if (periodOutput)
            *periodOutput = outPeriod;
        if (phase)
            *phase = (diff < inPeriod) ? diff : diff - inPeriod;

        return true;
    }

    static bool sampleVsyncPeriods(uint32_t *input, uint32_t *output)
    {
        int32_t inPeriod, outPeriod;

        if (!vsyncPeriodAndPhase(&inPeriod, &outPeriod, NULL))
            return false;

        *input = inPeriod;
        *output = outPeriod;

        return true;
    }

    // Find appropriate htotal that makes output frame time slightly more than the input.
    static bool findBestHTotal(uint32_t &bestHtotal)
    {
        uint16_t inHtotal = HSYNC_RST::read();
        uint32_t inPeriod, outPeriod;

        if (inHtotal == 0) {
            return false;
        } // safety
        if (!sampleVsyncPeriods(&inPeriod, &outPeriod)) {
            return false;
        }

        if (inPeriod == 0 || outPeriod == 0) {
            return false;
        } // safety

        // allow ~4 negative (inPeriod is < outPeriod) clock cycles jitter
        if ((inPeriod > outPeriod ? inPeriod - outPeriod : outPeriod - inPeriod) <= 4) {
            /*if (inPeriod >= outPeriod) {
        Serial.print("inPeriod >= out: ");
        Serial.println(inPeriod - outPeriod);
      }
      else {
        Serial.print("inPeriod < out: ");
        Serial.println(outPeriod - inPeriod);
      }*/
            bestHtotal = inHtotal;
        } else {
            // large htotal can push intermediates to 33 bits
            bestHtotal = (uint64_t)(inHtotal * (uint64_t)inPeriod) / (uint64_t)outPeriod;
        }

        // new 08.11.19: skip this step, IF period measurement should be stable enough to give repeatable results
        //if (bestHtotal == (inHtotal + 1)) { bestHtotal -= 1; } // works well
        //if (bestHtotal == (inHtotal - 1)) { bestHtotal += 1; } // check with SNES + vtotal = 1000 (1280x960)

#ifdef FS_DEBUG
        if (bestHtotal != inHtotal) {
            Serial.print(F("                     wants new htotal, oldbest: "));
            Serial.print(inHtotal);
            Serial.print(F(" newbest: "));
            Serial.println(bestHtotal);
            Serial.print(F("                     inPeriod: "));
            Serial.print(inPeriod);
            Serial.print(F(" outPeriod: "));
            Serial.println(outPeriod);
        }
#endif
        return true;
    }

public:
    // sets syncLockReady = ready() = false, which in turn starts a new init()
    // -> findBestHtotal() run in loop()
    static void reset(uint8_t frameTimeLockMethod)
    {
#ifdef FS_DEBUG
        Serial.print("FS reset(), with correction: ");
#endif
        if (syncLastCorrection != 0) {
#ifdef FS_DEBUG
            Serial.println("Yes");
#endif
            uint16_t vtotal = 0, vsst = 0;
            VRST_SST::read(vtotal, vsst);
            uint16_t timeout = 0;
            vtotal -= syncLastCorrection;
            if (frameTimeLockMethod == 0) { // moves VS position
                vsst -= syncLastCorrection;
            }

            while ((GBS::STATUS_VDS_FIELD::read() == 1) && (++timeout < 400))
                ;
            GBS::VDS_VS_ST::write(vsst);
            timeout = 0;
            while ((GBS::STATUS_VDS_FIELD::read() == 0) && (++timeout < 400))
                ;
            GBS::VDS_VSYNC_RST::write(vtotal);
        }
#ifdef FS_DEBUG
        else {
            Serial.println("No");
        }
#endif
        fsDebugPrintf("FrameSyncManager::reset(%d)\n", frameTimeLockMethod);

        syncLockReady = false;
        syncLastCorrection = 0;
        delayLock = 0;
        wireCalNeeded[0] = wireCalNeeded[1] = true;   // a new mode: the wires' offsets to the pin are measured again
        // Don't clear maybeFreqExt_per_videoFps.
        //
        // Clearing is unsafe, since many callers call reset(), don't
        // call externalClockGenSyncInOutRate() -> initFrequency(), then
        // expect runFrequency() to keep working.
        //
        // Not clearing is hopefully safe, since when loading an output
        // resolution, externalClockGenResetClock() calls
        // FrameSync::clearFrequency() and clears the variable, and
        // later someone calls externalClockGenSyncInOutRate() ->
        // FrameSync::initFrequency().
    }

    static void resetWithoutRecalculation()
    {
        syncLockReady = false;
        delayLock = 0;
    }

    static uint16_t init()
    {
        uint32_t bestHTotal = 0;

        // Adjust output horizontal sync timing so that the overall
        // frame time is as close to the input as possible while still
        // being less.  Increasing the vertical frame size slightly
        // should then push the output frame time to being larger than
        // the input.
        if (!findBestHTotal(bestHTotal)) {
            return 0;
        }

        syncLockReady = true;
        delayLock = 0;
        return (uint16_t)bestHTotal;
    }

    static uint32_t getPulseTicks()
    {
        uint32_t inStart, inStop;
        if (!vsyncInputSample(&inStart, &inStop)) {
            return 0;
        }
        return inStop - inStart;
    }

    static bool ready(void)
    {
        return syncLockReady;
    }

    // locked and the clock/FPS mapping known (ready() alone can be true right after a preset change)
    static bool frequencyReady(void)
    {
        return syncLockReady && maybeFreqExt_per_videoFps >= 0;
    }

    static int16_t getSyncLastCorrection()
    {
        return syncLastCorrection;
    }

    static void cleanup()
    {
        fsDebugPrintf("FrameSyncManager::cleanup(), resetting video frequency\n");

        syncLastCorrection = 0; // the important bit
        syncLockReady = 0;
        delayLock = 0;

        // Should we clear maybeFreqExt_per_videoFps?
        //
        // Clearing is hopefully safe. cleanup() appears to only be
        // called when switching between 15 kHz and 31 kHz inputs, or
        // when no video is present for an extended period of time and
        // the output shuts off. (cleanup() is not called when switching
        // between 240p and 480i.) When a new video signal is present,
        // someone calls externalClockGenSyncInOutRate() ->
        // FrameSync::initFrequency() to reinitialize the output frame
        // sync.
        //
        // Not clearing is hopefully safe. See reset() for an
        // explanation.
        maybeFreqExt_per_videoFps = -1;
    }

    // Sample vsync start and stop times from debug pin.
    static bool vsyncInputSample(uint32_t *start, uint32_t *stop)
    {
        yield();
        ESP.wdtDisable();
        MeasurePeriod::start();

        // Two rising edges = one period. Bounded in time, not in loop passes: with no edges on the debug pin (a broken
        // or unwired DEBUG_IN_PIN) the old 3,000,000 passes kept the ESP away from WiFi for about a second per call.
        const uint32_t t0 = ESP.getCycleCount();
        const uint32_t limit = sampleTimeoutCycles();
        while (ESP.getCycleCount() - t0 < limit) {
            if (MeasurePeriod::armed) {
                MeasurePeriod::armed = 0;
                delay(7);
                WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
            }
            if (MeasurePeriod::stopTime > 0) {
                break;
            }
        }
        MeasurePeriod::stop();
        *start = MeasurePeriod::startTime;
        *stop = MeasurePeriod::stopTime;
        ESP.wdtEnable(0);
        WiFi.setSleepMode(WIFI_NONE_SLEEP);

        if ((*start >= *stop) || *stop == 0 || *start == 0) {
            // ESP.getCycleCount() overflow oder no pulse, just fail this round
            return false;
        }

        return true;
    }

    // Perform vsync phase locking.  This is accomplished by measuring
    // the period and phase offset of the input and output vsync
    // signals and adjusting the frame size (and thus the output vsync
    // frequency) to bring the phase offset closer to the desired
    // value.
    static bool runVsync(uint8_t frameTimeLockMethod)
    {
        int32_t period;
        int32_t phase;
        int32_t target;
        int16_t correction;

        if (!syncLockReady)
            return false;

        if (delayLock < 2) {
            delayLock++;
            return true;
        }

        if (!vsyncPeriodAndPhase(&period, NULL, &phase))
            return false;

        target = (syncTargetPhase * period) / 360;

        if (phase > target)
            correction = 0;
        else
            correction = syncCorrection;

#ifdef FS_DEBUG
        Serial.printf_P(PSTR("phase: %7d target: %7d"), phase, target);
        if (correction == syncLastCorrection) {
            // terminate line if returning early
            Serial.println();
        }
#endif
#ifdef FS_DEBUG_LED
        if (correction == 0) {
            digitalWrite(LED_BUILTIN, LOW); // LED ON
        } else {
            digitalWrite(LED_BUILTIN, HIGH); // LED OFF
        }
#endif

        // return early?
        if (correction == syncLastCorrection) {
            return true;
        }

        int16_t delta = correction - syncLastCorrection;
        uint16_t vtotal = 0, vsst = 0;
        uint16_t timeout = 0;
        VRST_SST::read(vtotal, vsst);
        vtotal += delta;
        if (frameTimeLockMethod == 0) { // moves VS position
            vsst += delta;
        }
        // else it is method 1: leaves VS position alone

        while ((GBS::STATUS_VDS_FIELD::read() == 1) && (++timeout < 400))
            ;
        GBS::VDS_VS_ST::write(vsst);
        timeout = 0;
        while ((GBS::STATUS_VDS_FIELD::read() == 0) && (++timeout < 400))
            ;
        GBS::VDS_VSYNC_RST::write(vtotal);

        syncLastCorrection = correction;

#ifdef FS_DEBUG
        Serial.printf_P(PSTR("  vtotal: %4d\n"), vtotal);
#endif

        return true;
    }

    static void clearFrequency() {
        maybeFreqExt_per_videoFps = -1;
        smoothedFpsInput = 0;
    }

    static void initFrequency(float outFramesPerS, uint32_t freqExtClockGen) {
        /*
        This value can be interpreted in multiple ways:

        - Each output frame is a fixed number of video clocks long, at a
          given output resolution.
        - At a given output resolution, the video clock rate should be
          proportional to the input FPS.
        */
        maybeFreqExt_per_videoFps = (float)freqExtClockGen / outFramesPerS;
    }

    // Perform vsync phase locking.  This is accomplished by measuring
    // the period and phase offset of the input and output vsync
    // signals, then adjusting the output video clock to bring the phase
    // offset closer to the desired value.
    static bool runFrequency()
    {
        if (maybeFreqExt_per_videoFps < 0) {
            SerialM.printf_P(
                PSTR("Error: trying to tune external clock frequency while clock frequency uninitialized!\n"));
            return true;
        }

        // Compare to externalClockGenSyncInOutRate().
        if (GBS::PAD_CKIN_ENZ::read() != 0) {
            // Failed due to external factors (PAD_CKIN_ENZ=0 on
            // startup), not bad input signal, don't return frame sync
            // error.
            fsDebugPrintf(
                "Skipping FrameSyncManager::runFrequency(), GBS::PAD_CKIN_ENZ::read() != 0\n");
            return true;
        }

        if (rto->outModeHdBypass) {
            fsDebugPrintf(
                "Skipping FrameSyncManager::runFrequency(), rto->outModeHdBypass\n");
            return true;
        }
        if (GBS::PLL648_CONTROL_01::read() != 0x75) {
            SerialM.printf_P(
                PSTR("Error: trying to tune external clock frequency while set to internal clock, PLL648_CONTROL_01=%d!\n"),
                GBS::PLL648_CONTROL_01::read());
            return true;
        }

        if (!syncLockReady) {
            fsDebugPrintf(
                "Skipping FrameSyncManager::runFrequency(), !syncLockReady\n");
            return false;
        }

        // ESP32 FPU only accelerates single-precision float add/mul, not divide, not double.
        // https://esp32.com/viewtopic.php?p=82090#p82090

        // ESP CPU cycles/s
        const float esp8266_clock_freq = ESP.getCpuFreqMHz() * 1000000;

        // ESP CPU cycles
        int32_t periodInput;  // int32_t periodOutput;
        int32_t phase;

        // Frame/s
        float fpsInput;

        // Measure input period until we get two consistent measurements. This
        // substantially reduces the chance of incorrectly guessing FPS when
        // input sync changes (but does not eliminate it, eg. when resetting a
        // SNES).
        bool success = false;
        for (int attempt = 0; attempt < 2; attempt++) {
            // Measure input period and output latency.
            bool ret = vsyncPeriodAndPhase(&periodInput, nullptr, &phase);   // gives the test bus back as found
            if (!ret) {
                SerialM.printf_P(PSTR("runFrequency(): vsyncPeriodAndPhase failed, retrying...\n"));
                continue;
            }

            fpsInput = esp8266_clock_freq / (float)periodInput;
            if (fpsInput < 47.0f || fpsInput > 86.0f) {
                SerialM.printf_P(
                    PSTR("runFrequency(): fpsInput wrong: %f, retrying...\n"),
                    fpsInput);
                continue;
            }
            if (lastWasWire) {   // the wires' period is already a mean of consecutive fields: no second look
                success = true;
                break;
            }

            // Measure input period again: getPulseTicks() -> vsyncInputSample() needs GBS::TEST_BUS_SEL = 0
            const uint8_t busSel = GBS::TEST_BUS_SEL::read();
            if (busSel != 0x0) GBS::TEST_BUS_SEL::write(0x0);
            uint32_t periodInput2 = getPulseTicks();
            if (busSel != 0x0) GBS::TEST_BUS_SEL::write(busSel);
            if (periodInput2 == 0) {
                SerialM.printf_P(PSTR("runFrequency(): getPulseTicks failed, retrying...\n"));
                continue;
            }
            float fpsInput2 = esp8266_clock_freq / (float)periodInput2;
            if (fpsInput2 < 47.0f || fpsInput2 > 86.0f) {
                SerialM.printf_P(
                    PSTR("runFrequency(): fpsInput2 wrong: %f, retrying...\n"),
                    fpsInput2);
                continue;
            }

            // Check that the two FPS measurements are sufficiently close.
            float diff = fabs(fpsInput2 - fpsInput);
            float relDiff = diff / std::min(fpsInput, fpsInput2);
            if (relDiff != relDiff || diff > 0.5f || relDiff > 0.00833f) {
                SerialM.printf_P(
                    PSTR("FrameSyncManager::runFrequency() measured inconsistent FPS %f and %f, retrying...\n"),
                    fpsInput,
                    fpsInput2);
                continue;
            }

            fpsInput = 0.5f * (fpsInput + fpsInput2);   // the two consistent measurements averaged (gbs-control-nx)
            success = true;
            break;
        }
        if (!success) {
            SerialM.printf_P(PSTR("FrameSyncManager::runFrequency() failed!\n"));
            return false;
        }
        // EMA (alpha 0.15) on the input FPS, the smoothed value drives the correction (gbs-control-nx). Measured
        // with frame lock on (Mega Drive 240p, 86 corrections each): output FPS sd 0.5 -> 0.1 ppm, mean clock step
        // 89 -> 13 Hz, phase error not worse (max 0.0001 frame). Reseeded by clearFrequency().
        if (smoothedFpsInput < 47.0f) smoothedFpsInput = fpsInput;
        smoothedFpsInput = 0.15f * fpsInput + 0.85f * smoothedFpsInput;
        fpsInput = smoothedFpsInput;

        // ESP CPU cycles
        int32_t target = (syncTargetPhase * periodInput) / 360;

        // Latency error (distance behind target), in fractional frames.
        // If latency increases, boost frequency, and vice versa.
        const float latency_err_frames =
            (float)(phase - target) // cycles
            / esp8266_clock_freq // s
            * fpsInput; // frames

        // 0.0038f is 2/525, the difference between SNES and Wii 240p.
        // This number is somewhat arbitrary, but works well in
        // practice.
        float correction = 0.0038f * latency_err_frames;

        // Some LCD displays (eg. Dell U2312HM) lose sync when changing
        // frequency by 0.1% (switching between 59.94 and 60 FPS).
        //
        // To ensure long-term FPS stability, clamp the maximum deviation from
        // input FPS to 0.06%. This is sufficient as long as fpsInput does not
        // vary drastically from frame to frame.
        constexpr float MAX_CORRECTION = 0.0006f;
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

        const float rawFpsOutput = fpsInput * (1 + correction);

        // This has floating-point conversion round-trip rounding errors, which
        // is suboptimal, but it's not a big deal.
        const float prevFpsOutput = (float)rto->freqExtClockGen / maybeFreqExt_per_videoFps;

        // In case fpsInput is measured incorrectly, rawFpsOutput may be
        // drastically different from the previous frame's output FPS. To limit
        // the impact of incorrect input FPS measurements, clamp the maximum FPS
        // deviation relative to the previous frame's *output* FPS. This
        // provides short-term FPS stability.
        constexpr float MAX_FPS_CHANGE = 0.0006f;
        float fpsOutput = rawFpsOutput;
        fpsOutput = std::min(fpsOutput, prevFpsOutput * (1 + MAX_FPS_CHANGE));
        fpsOutput = std::max(fpsOutput, prevFpsOutput * (1 - MAX_FPS_CHANGE));

        if (fabs(rawFpsOutput - prevFpsOutput) >= 1.f) {
            SerialM.printf_P(
                PSTR("FPS excursion detected! Measured input FPS %f, previous output FPS %f"),
                fpsInput, prevFpsOutput);
        }

        fsDebugPrintf(
            "periodInput=%d, fpsInput=%f, latency_err_frames=%f from %f, "
            "fpsOutput=%f := %f\n",
            periodInput, fpsInput, latency_err_frames, (float)syncTargetPhase / 360.f,
            prevFpsOutput, fpsOutput);

        const auto freqExtClockGen = (uint32_t)(maybeFreqExt_per_videoFps * fpsOutput);

        fsDebugPrintf(
            "Setting clock frequency from %u to %u\n",
            rto->freqExtClockGen, freqExtClockGen);

        setExternalClockGenFrequencySmooth(freqExtClockGen);
        return true;
    }
};

// grrrrrrrrr

template <class GBS, class Attrs>
int16_t FrameSyncManager<GBS, Attrs>::syncLastCorrection;

template <class GBS, class Attrs>
float FrameSyncManager<GBS, Attrs>::maybeFreqExt_per_videoFps;

template <class GBS, class Attrs>
uint8_t FrameSyncManager<GBS, Attrs>::delayLock;

template <class GBS, class Attrs>
bool FrameSyncManager<GBS, Attrs>::syncLockReady;

template <class GBS, class Attrs>
float FrameSyncManager<GBS, Attrs>::smoothedFpsInput;

template <class GBS, class Attrs>
bool FrameSyncManager<GBS, Attrs>::wireCalNeeded[2] = {true, true};

template <class GBS, class Attrs>
int32_t FrameSyncManager<GBS, Attrs>::wireOffset[2];

template <class GBS, class Attrs>
bool FrameSyncManager<GBS, Attrs>::lastWasWire;

template <class GBS, class Attrs>
uint8_t FrameSyncManager<GBS, Attrs>::wireFlags;
#endif