/**
 * @file  i2c_probe.c
 * @brief Passive probe on the ESP's I2C bus (see i2c_probe.h).
 */
#include "i2c_probe.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>

/* Probe lines go to the USB console port only (the debug UART can't keep up); dropped while nobody listens */
static const ConsolePort *s_port;

void I2CProbe_SetPort(const ConsolePort *usb) { s_port = usb; }

static size_t portFree(void) { return (s_port && s_port->connected()) ? s_port->space() : 0U; }

static void out(const char *fmt, ...)
{
    if (!s_port || !s_port->connected()) return;
    char b[256];   /* the ps line is ~230 characters */
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(b) - 1) n = (int)sizeof(b) - 1;
    s_port->write(b, (size_t)n);
}

#define PROBE_SCL_PIN     (1U << 4U)   /* PA4 */
#define PROBE_SDA_PIN     (1U << 5U)   /* PA5 */
#define PROBE_SCL_IRQn    (INT011_IRQn)   /* INT003 link UART, INT006 vsync wires, INT007 tick, INT008-010 I2C slave */
#define PROBE_SDA_IRQn    (INT012_IRQn)
/* Above the I2C slave (05) and the vsync wires (04): a probe edge must be served within ~1 us. The slave's
 * hardware stretches SCL while its interrupt waits, so a short delay there is harmless. */
#define PROBE_IRQ_PRIO    (DDL_IRQ_PRIO_03)
#define OWN_ADDR          (0x32U)

#define REC_DATA          (34U)        /* register + 32 data bytes (the ESP's Wire chunks) */
#define REC_COUNT         (512U)       /* power of two; a preset load is a few hundred writes in
                                          * ~100 ms, printed over ~0.3 s */

#define F_ADDR_NACK       (0x01U)
#define F_DATA_NACK       (0x02U)
#define F_TRUNC           (0x04U)
#define F_PARTIAL         (0x08U)      /* ended mid-byte: lost edges or noise */
#define F_RESTART         (0x10U)      /* ended by a repeated START */
#define F_ADDR_SEEN       (0x80U)      /* internal: the address byte has been received */

typedef struct {
    uint32_t cycles;                   /* DWT cycle counter at the end */
    uint8_t  addr;                     /* address byte: 7-bit address << 1 | R/W */
    uint8_t  nbytes;                   /* bytes after the address (all, even if not stored) */
    uint8_t  flags;
    uint8_t  len;                      /* stored bytes */
    uint8_t  data[REC_DATA];
} probe_rec_t;

static probe_rec_t s_ring[REC_COUNT];
static volatile uint32_t s_head, s_tail;   /* ISR writes head, task reads tail */

static volatile uint8_t  s_on;
static uint8_t s_mode;                     /* PROBE_MODE_* */
static volatile uint8_t  s_active;         /* between START and STOP */
static volatile uint8_t  s_nbits;
static volatile uint16_t s_shift;
static probe_rec_t s_cur;

static volatile uint32_t s_transfers, s_reads, s_pointer, s_addrOnly, s_own, s_logged,
                         s_dropped, s_partial, s_nacks, s_changes, s_oledBytes, s_osdRam, s_truncated;

/* "Blind": while the HC32 erases or programs its own flash the CPU stalls on instruction
 * fetches, interrupts included, and the probe misses edges (seen as ?partial right after
 * "[FLASH] Settings saved"). A garbled transfer that overlaps such a period is the probe's
 * fault, not the bus's: count it as blind instead of reporting it. */
static volatile uint8_t s_blindDepth;
static volatile uint32_t s_blindPeriods, s_blind;
static uint32_t s_curBlindStamp;

void I2CProbe_SetBlind(uint8_t blind)
{
    if (blind) { s_blindDepth++; s_blindPeriods++; }
    else if (s_blindDepth) s_blindDepth--;
}

/* Read histogram ("ph"): which registers the ESP polls. A read is a 1-byte register-pointer
 * write and then a read transfer; the pointer write says which register. The TV5725's
 * segment is tracked here from writes to 0xF0. */
#define HIST_TV_SEGS (7U)                       /* 6 = segment not seen yet */
static uint32_t s_histTv[HIST_TV_SEGS][256];    /* pointer writes per TV5725 segment/register */
static uint32_t s_histOther[128];               /* pointer writes per other chip address */
static uint32_t s_readsPerChip[128];
static uint8_t s_isrTvSeg = 0xFFU;

static void hist_note(void)
{
    const uint8_t a7 = (uint8_t)(s_cur.addr >> 1);
    if (s_cur.addr & 1U) { s_readsPerChip[a7 & 0x7FU]++; return; }
    if (a7 == 0x17U && s_cur.len >= 2U && s_cur.data[0] == 0xF0U) { s_isrTvSeg = s_cur.data[1]; return; }
    if (s_cur.nbytes != 1U || s_cur.len < 1U) return;
    if (a7 == 0x17U) s_histTv[s_isrTvSeg < 6U ? s_isrTvSeg : 6U][s_cur.data[0]]++;
    else s_histOther[a7 & 0x7FU]++;
}

void I2CProbe_PrintHistogram(void)
{
    /* Top 24 TV5725 registers by pointer writes (= reads), then the other chips */
    uint8_t shown[HIST_TV_SEGS][32] = {{0}};
    out("[PROBE] reads by register (TV5725 segment_register: count since p1/p2)\r\n");
    for (uint8_t n = 0U; n < 24U; n++) {
        uint32_t best = 0U; uint8_t bs = 0U; uint16_t br = 0U;
        for (uint8_t sg = 0U; sg < HIST_TV_SEGS; sg++)
            for (uint16_t r = 0U; r < 256U; r++)
                if (!((shown[sg][r >> 3] >> (r & 7U)) & 1U) && s_histTv[sg][r] > best) { best = s_histTv[sg][r]; bs = sg; br = r; }
        if (best == 0U) break;
        shown[bs][br >> 3] |= (uint8_t)(1U << (br & 7U));
        if (bs < 6U) out("  TV %u_%02X %lu\r\n", (unsigned)bs, (unsigned)br, (unsigned long)best);
        else out("  TV ?_%02X %lu\r\n", (unsigned)br, (unsigned long)best);
    }
    for (uint8_t a = 0U; a < 128U; a++)
        if (s_readsPerChip[a] || s_histOther[a])
            out("  chip %02X: reads %lu, pointer writes %lu\r\n", (unsigned)a, (unsigned long)s_readsPerChip[a],
                   (unsigned long)s_histOther[a]);
}

static void finish(uint8_t flag)
{
    /* A STOP or repeated START is preceded by one more SCL rise (SDA held for the
     * condition), so one stray bit is normal; more means lost edges. */
    if (s_nbits > 1U) s_cur.flags |= F_PARTIAL;
    s_cur.flags |= flag;
    if ((s_cur.flags & F_PARTIAL) && (s_blindDepth || s_curBlindStamp != s_blindPeriods)) {
        s_blind++;            /* edges lost while the HC32 wrote its flash */
        return;
    }
    s_cur.cycles = DWT->CYCCNT;
    s_transfers++;
    if (!(s_cur.flags & (F_ADDR_NACK | F_PARTIAL))) hist_note();
    /* Only writes (and anything odd) go to the ring; the ESP polls the scaler thousands
     * of times a second, so reads and register-pointer writes are just counted here. */
    const uint8_t bad = s_cur.flags & (F_ADDR_NACK | F_DATA_NACK | F_PARTIAL);
    if ((s_cur.addr >> 1) == OWN_ADDR) { s_own++; return; }
    if (!bad) {
        if (s_cur.addr & 1U) { s_reads++; return; }
        if (s_cur.nbytes == 0U) { s_addrOnly++; return; }
        if (s_cur.nbytes == 1U) { s_pointer++; return; }
    }
    const uint32_t next = (s_head + 1U) & (REC_COUNT - 1U);
    if (next == s_tail) {
        s_dropped++;
    } else {
        s_ring[s_head] = s_cur;
        s_head = next;
    }
}

static void begin(void)
{
    s_curBlindStamp = s_blindDepth ? s_blindPeriods - 1U : s_blindPeriods;
    s_active = 1U;
    s_nbits = 0U;
    s_shift = 0U;
    s_cur.addr = 0U;
    s_cur.nbytes = 0U;
    s_cur.flags = 0U;
    s_cur.len = 0U;
}

/* SCL rising edge: sample SDA. 9 bits = a byte and its ACK (0) / NACK (1). */
static void ProbeScl_IrqCallback(void)
{
    const uint16_t in = CM_GPIO->PIDRA;
    CM_INTC->EIFCR = EXTINT_CH04;
    if (!s_active) return;
    s_shift = (uint16_t)((s_shift << 1) | ((in & PROBE_SDA_PIN) ? 1U : 0U));
    if (++s_nbits < 9U) return;
    const uint8_t b = (uint8_t)(s_shift >> 1);
    const uint8_t nack = (uint8_t)(s_shift & 1U);
    s_nbits = 0U;
    s_shift = 0U;
    if (!(s_cur.flags & F_ADDR_SEEN)) {
        s_cur.addr = b;
        s_cur.flags |= F_ADDR_SEEN;
        if (nack) s_cur.flags |= F_ADDR_NACK;
        return;
    }
    if (s_cur.nbytes < 255U) s_cur.nbytes++;
    if (!(s_cur.addr & 1U)) {          /* write: keep the data */
        if (s_cur.len < REC_DATA) s_cur.data[s_cur.len++] = b;
        else s_cur.flags |= F_TRUNC;
        if (nack) s_cur.flags |= F_DATA_NACK;
    }
}

/* SDA edge while SCL is high: falling = START (or repeated START), rising = STOP */
static void ProbeSda_IrqCallback(void)
{
    const uint16_t in = CM_GPIO->PIDRA;
    CM_INTC->EIFCR = EXTINT_CH05;
    if (!(in & PROBE_SCL_PIN)) return;     /* data changing while SCL is low */
    if (!(in & PROBE_SDA_PIN)) {
        if (s_active) finish(F_RESTART);
        begin();
    } else if (s_active) {
        finish(0U);
        s_active = 0U;
    }
}

void I2CProbe_Init(void)
{
    /* Cycle counter for timestamps */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    stc_extint_init_t ext;
    (void)EXTINT_StructInit(&ext);
    ext.u32Filter = EXTINT_FILTER_OFF;
    ext.u32Edge = EXTINT_TRIG_RISING;
    (void)EXTINT_Init(EXTINT_CH04, &ext);
    ext.u32Edge = EXTINT_TRIG_BOTH;
    (void)EXTINT_Init(EXTINT_CH05, &ext);

    stc_irq_signin_config_t irq;
    irq.enIRQn = PROBE_SCL_IRQn;
    irq.enIntSrc = INT_SRC_PORT_EIRQ4;
    irq.pfnCallback = &ProbeScl_IrqCallback;
    (void)INTC_IrqSignIn(&irq);
    irq.enIRQn = PROBE_SDA_IRQn;
    irq.enIntSrc = INT_SRC_PORT_EIRQ5;
    irq.pfnCallback = &ProbeSda_IrqCallback;
    (void)INTC_IrqSignIn(&irq);
    NVIC_SetPriority(PROBE_SCL_IRQn, PROBE_IRQ_PRIO);
    NVIC_SetPriority(PROBE_SDA_IRQn, PROBE_IRQ_PRIO);
    s_on = 0U;
}

void I2CProbe_Enable(uint8_t mode)
{
    const uint8_t on = (mode != PROBE_MODE_OFF) ? 1U : 0U;
    if (on && s_mode != mode) I2CProbe_Forget();
    s_mode = mode;
    /* Only the pins' external-interrupt enable bit changes; their function (I2C2) and
     * drive settings stay as I2CV_Init() left them. */
    if (on) {
        s_active = 0U;
        s_head = s_tail = 0U;
        s_transfers = s_reads = s_pointer = s_addrOnly = s_own = s_logged = 0U;   /* per run */
        s_dropped = s_partial = s_nacks = s_changes = s_oledBytes = s_osdRam = s_truncated = 0U;
        s_blind = 0U;
        for (uint32_t i = 0U; i < HIST_TV_SEGS * 256U; i++) ((uint32_t *)s_histTv)[i] = 0U;
        for (uint32_t i = 0U; i < 128U; i++) s_histOther[i] = s_readsPerChip[i] = 0U;
        s_isrTvSeg = 0xFFU;
        CM_INTC->EIFCR = EXTINT_CH04 | EXTINT_CH05;
        SET_REG16_BIT(CM_GPIO->PCRA4, GPIO_PCR_INTE);
        SET_REG16_BIT(CM_GPIO->PCRA5, GPIO_PCR_INTE);
        NVIC_ClearPendingIRQ(PROBE_SCL_IRQn);
        NVIC_ClearPendingIRQ(PROBE_SDA_IRQn);
        NVIC_EnableIRQ(PROBE_SCL_IRQn);
        NVIC_EnableIRQ(PROBE_SDA_IRQn);
    } else {
        NVIC_DisableIRQ(PROBE_SCL_IRQn);
        NVIC_DisableIRQ(PROBE_SDA_IRQn);
        CLR_REG16_BIT(CM_GPIO->PCRA4, GPIO_PCR_INTE);
        CLR_REG16_BIT(CM_GPIO->PCRA5, GPIO_PCR_INTE);
        s_active = 0U;
    }
    s_on = on ? 1U : 0U;
    out("[PROBE] %s\r\n", mode == PROBE_MODE_CHANGES ? "on: register changes" :
                               mode == PROBE_MODE_RAW ? "on: raw writes" : "off");
}

/* ---- register copies ("changes" mode) ------------------------------------------------ */

#define ADDR_TV5725   (0x17U)
#define ADDR_SI5351   (0x60U)
#define ADDR_STV9426  (0x5DU)
#define ADDR_OLED     (0x3CU)
#define TV_SEGMENTS   (6U)

static uint8_t s_tv[TV_SEGMENTS][256], s_tvKnown[TV_SEGMENTS][32];
static uint8_t s_tvSeg = 0xFFU;                    /* segment register 0xF0; unknown */
static uint8_t s_clk[256], s_clkKnown[32];
static uint8_t s_osd[16], s_osdKnown[2];           /* STV9426 registers 0x3FF0-0x3FFF */
static uint8_t s_oledLog = 0U;                     /* po1: print OLED transfers in changes mode */

void I2CProbe_OledLog(uint8_t on)
{
    s_oledLog = on ? 1U : 0U;
    out("[PROBE] OLED transfers %s\r\n", s_oledLog ? "printed" : "counted only");
}

#define KNOWN(k, i)     (((k)[(i) >> 3] >> ((i) & 7U)) & 1U)
#define SET_KNOWN(k, i) ((k)[(i) >> 3] |= (uint8_t)(1U << ((i) & 7U)))

void I2CProbe_Forget(void)
{
    for (uint32_t i = 0U; i < sizeof(s_tvKnown); i++) ((uint8_t *)s_tvKnown)[i] = 0U;
    for (uint32_t i = 0U; i < sizeof(s_clkKnown); i++) s_clkKnown[i] = 0U;
    s_osdKnown[0] = s_osdKnown[1] = 0U;
    s_tvSeg = 0xFFU;
}

/* "C <ms> <chip> <reg> <old>><new>" for a register whose value changed (old ?? if the
 * probe hasn't seen it written since it was turned on or told to forget) */
static void change(uint32_t ms, const char *chip, const char *reg, uint8_t *val, uint8_t *known,
                   uint32_t idx, uint8_t v)
{
    if (KNOWN(known, idx) && val[idx] == v) return;
    if (KNOWN(known, idx)) out("C %lu %s %s %02X>%02X\r\n", (unsigned long)ms, chip, reg, val[idx], v);
    else out("C %lu %s %s ?" "?>%02X\r\n", (unsigned long)ms, chip, reg, v);  /* split: ??> is a trigraph */
    val[idx] = v;
    SET_KNOWN(known, idx);
    s_changes++;
}

static void raw(uint32_t ms, const probe_rec_t *r)
{
    out("%c %lu %02X", (r->flags & (F_ADDR_NACK | F_DATA_NACK | F_PARTIAL)) ? 'N' : 'W', (unsigned long)ms,
        (unsigned)(r->addr >> 1));
    if (r->flags & F_ADDR_NACK) out(" !A");
    for (uint8_t i = 0U; i < r->len; i++) out(" %02X", r->data[i]);
    if (r->flags & F_TRUNC) out(" +%u", (unsigned)(r->nbytes - r->len));
    if (r->flags & F_DATA_NACK) out(" !D");
    if (r->flags & F_PARTIAL) out(" ?partial");
    out("\r\n");
}

/* A clean write in "changes" mode; false if it isn't a chip with a register copy */
static uint8_t decode(uint32_t ms, const probe_rec_t *r)
{
    const uint8_t a7 = (uint8_t)(r->addr >> 1);
    char reg[8];
    if (a7 == ADDR_OLED) {                              /* counted; whole transfers with po1 */
        s_oledBytes += r->nbytes;
        if (s_oledLog) raw(ms, r);
        return 1U;
    }
    if (r->len < 2U) return 0U;
    if (a7 == ADDR_TV5725) {
        uint8_t at = r->data[0];
        for (uint8_t i = 1U; i < r->len; i++, at++) {
            if (at == 0xF0U) { s_tvSeg = r->data[i]; continue; }   /* segment select */
            if (s_tvSeg >= TV_SEGMENTS) { raw(ms, r); return 1U; }
            snprintf(reg, sizeof(reg), "%u_%02X", (unsigned)s_tvSeg, (unsigned)at);
            change(ms, "TV", reg, s_tv[s_tvSeg], s_tvKnown[s_tvSeg], at, r->data[i]);
        }
        return 1U;
    }
    if (a7 == ADDR_SI5351) {
        uint8_t at = r->data[0];
        for (uint8_t i = 1U; i < r->len; i++, at++) {
            snprintf(reg, sizeof(reg), "%02X", (unsigned)at);
            change(ms, "CLK", reg, s_clk, s_clkKnown, at, r->data[i]);
        }
        return 1U;
    }
    if (a7 == ADDR_STV9426 && r->len >= 3U) {           /* address low, high (bank), data... */
        uint16_t at = (uint16_t)(r->data[0] | ((uint16_t)r->data[1] << 8));
        for (uint8_t i = 2U; i < r->len; i++, at++) {
            if (at >= 0x3FF0U && at <= 0x3FFFU) {
                snprintf(reg, sizeof(reg), "%04X", (unsigned)at);
                change(ms, "OSD", reg, s_osd, s_osdKnown, at - 0x3FF0U, r->data[i]);
            } else {
                s_osdRam++;                             /* character / attribute RAM */
            }
        }
        return 1U;
    }
    return 0U;
}

void I2CProbe_Task(void)
{
    static uint32_t lastCycles = 0U, msHigh = 0U;
    /* A few records per pass so the CLI and relay stay responsive, and only while the USB
     * ring has room: waiting here keeps lines in order instead of losing them */
    for (uint8_t n = 0U; n < 8U && s_tail != s_head && portFree() > 200U; n++) {
        const probe_rec_t r = s_ring[s_tail];
        s_tail = (s_tail + 1U) & (REC_COUNT - 1U);
        if (r.cycles < lastCycles) msHigh += 21475U;   /* 2^32 cycles at 200 MHz, in ms */
        lastCycles = r.cycles;
        const uint32_t ms = msHigh + r.cycles / 200000U;

        const uint8_t bad = r.flags & (F_ADDR_NACK | F_DATA_NACK | F_PARTIAL);
        if (r.flags & F_PARTIAL) s_partial++;
        if (r.flags & (F_ADDR_NACK | F_DATA_NACK)) s_nacks++;
        if (r.flags & F_TRUNC) s_truncated++;
        s_logged++;
        if (s_mode == PROBE_MODE_CHANGES && !bad && !(r.flags & F_TRUNC) && decode(ms, &r)) continue;
        raw(ms, &r);
    }
}

void I2CProbe_PrintStats(void)
{
    out("[PROBE] %s transfers %lu: writes %lu (changes %lu), reads %lu, pointer %lu, addr-only %lu, "
           "own 0x32 %lu, oled bytes %lu, osd ram %lu | nacks %lu, partial %lu, truncated %lu, dropped %lu, blind %lu\r\n",
           s_on ? "on" : "off", (unsigned long)s_transfers, (unsigned long)s_logged,
           (unsigned long)s_changes, (unsigned long)s_reads, (unsigned long)s_pointer,
           (unsigned long)s_addrOnly, (unsigned long)s_own, (unsigned long)s_oledBytes,
           (unsigned long)s_osdRam, (unsigned long)s_nacks, (unsigned long)s_partial,
           (unsigned long)s_truncated, (unsigned long)s_dropped, (unsigned long)s_blind);
}

/* "J" line: the bus was held stuck (SDA low, SCL high) and the I2C slave (i2c_vsync.c) clocked it free. With the
 * probe on, the transfer that was in progress says which chip held SDA and where the ESP stopped. */
void I2CProbe_ReportStuck(uint32_t stuckMs, uint8_t clocks, uint8_t released)
{
    const uint32_t ms = DWT->CYCCNT / 200000U;
    if (!s_on) {
        out("J %lu bus stuck %lu ms: SDA low, SCL high; %u clocks, %s (p1/p2 for the transfer in progress)\r\n",
            (unsigned long)ms, (unsigned long)stuckMs, (unsigned)clocks, released ? "released" : "STILL LOW");
        return;
    }
    out("J %lu bus stuck %lu ms: SDA low, SCL high; %u clocks, %s; in progress: %s", (unsigned long)ms,
        (unsigned long)stuckMs, (unsigned)clocks, released ? "released" : "STILL LOW", s_active ? "yes" : "no");
    if (s_active) {
        if (s_cur.flags & F_ADDR_SEEN)
            out(", %02X %s, byte %u bit %u", (unsigned)(s_cur.addr >> 1), (s_cur.addr & 1U) ? "read" : "write",
                (unsigned)s_cur.nbytes, (unsigned)s_nbits);
        else
            out(", address bit %u", (unsigned)s_nbits);
    }
    out("\r\n");
    s_active = 0U;
}
