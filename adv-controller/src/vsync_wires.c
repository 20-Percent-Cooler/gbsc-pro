/* Vsync wires (hardware mod, optional): the TV5725's output vsync and the LM1881's source vsync timed by the HC32, so
 * the ESP's frame lock can read the periods and the phase from the link (ADV_CMD_VSYNC) instead of sampling the
 * TV5725 debug pin with the CPU.
 *
 *  Output: VS_OUT at the STV9426's V-SYNC pin (U5 pin 2) -> 1k -> J18 pin 4 = PB4 (JTAG NJTRST, released)
 *  Source: LM1881 VSOUT (U12 pin 3)                       -> 1k -> J18 pin 3 = PB5
 *
 * Both pins are Timer A 6 capture inputs (function 5: PB4 TIMA_6_PWM6, PB5 TIMA_6_PWM7); the timer only runs so that
 * its capture logic raises the interrupt, every edge is stamped with the 200 MHz cycle counter (DWT). Pull-ups: a
 * board without the wires reads idle levels, no edges, flags 0.
 *
 * Source: the falling edge (start of the LM1881's active-low vsync). Two consecutive periods are averaged: 480i
 * fields are 262/263 lines and some sources' field lengths alternate.
 * Output: both edges, the pin level read in the interrupt. Some outputs carry line-rate pulses on this wire (480p's
 * wide hsync, ~2.5 us), so the frame mark is the first edge after an idle level longer than 2 ms when the last second
 * had such levels (every clean output), else the start of a 40 us..2 ms level reaching half the longest of the last
 * second (the vsync pulse among line pulses). */
#include "vsync_wires.h"
#include "main.h"

#define VSW_TMRA        (CM_TMRA_6)
#define VSW_IRQn        (INT006_IRQn)     /* INT007: the system tick (TMR0_2), INT003: link UART errors */
#define CYC_PER_SEC     (200000000UL)
#define PRESENT_CYC     (CYC_PER_SEC / 10U)        /* "running": an edge in the last 100 ms */
#define VS_MIN_CYC      (40U * 200U)               /* 40 us */
#define VS_MAX_CYC      (2000U * 200U)             /* 2 ms */
#define IDLE_PER_SEC    (20U)
#define RING            (16U)                      /* periods averaged (~0.27 s at 60 Hz) */

typedef struct {
    volatile uint32_t last, edges, prev;   /* last edge, edge count, previous raw in-range period (0 none) */
    volatile uint32_t ring[RING];
    volatile uint8_t w, n;                 /* ring write index, entries */
    volatile uint32_t perAt;               /* when the last period was added */
    uint8_t pair;                          /* average two consecutive periods (source) */
} vsw_chan_t;

typedef struct {
    volatile uint32_t start, refPh;
    volatile uint8_t high, valid, refOk;
    volatile uint32_t maxCur, maxPrev, idleCur, idlePrev, secAt;
} vsw_lvl_t;

static vsw_chan_t s_src, s_out;
static vsw_lvl_t s_lvl;
static volatile uint32_t s_phase, s_phaseAt;
static volatile uint8_t s_seq, s_phaseOk;
static uint8_t s_on;

static void chan_edge(vsw_chan_t *c, uint32_t now)
{
    if (c->edges) {
        const uint32_t p = now - c->last;
        if (p > CYC_PER_SEC / 87U && p < CYC_PER_SEC / 39U) {   /* 40..86 Hz */
            const uint32_t q = (c->pair && c->prev) ? (p + c->prev) / 2U : p;
            c->prev = p;
            c->ring[c->w] = q;
            c->w = (uint8_t)((c->w + 1U) % RING);
            if (c->n < RING) c->n++;
            c->perAt = now;
        } else {
            c->prev = 0U;
            c->n = 0U;                                          /* a gap: the old periods are another signal's */
        }
    }
    c->last = now;
    c->edges++;
}

static void out_mark(uint32_t at, uint8_t phOk, uint32_t ph)
{
    chan_edge(&s_out, at);
    s_seq++;
    s_phaseOk = phOk;
    s_phase = ph;
    s_phaseAt = at;
}

/* the second's bookkeeping for the level logic, from the edges themselves (no task needed) */
static void lvl_second(vsw_lvl_t *L, uint32_t now)
{
    if (now - L->secAt < CYC_PER_SEC) return;
    L->maxPrev = L->maxCur; L->maxCur = 0U;
    L->idlePrev = L->idleCur; L->idleCur = 0U;
    L->secAt = now;
}

static void out_edge(vsw_lvl_t *L, uint32_t now, uint8_t high)
{
    if (high == L->high && L->valid) return;                 /* both edges of a short pulse in one call */
    lvl_second(L, now);
    const uint32_t start = L->start, len = now - start;
    if (L->valid) {
        const uint8_t byIdle = L->idlePrev >= IDLE_PER_SEC;
        if (len > VS_MAX_CYC) {
            L->idleCur++;
            if (byIdle) {                                    /* an idle level ended: this edge starts the vsync */
                const uint32_t ph = now - s_src.last;
                out_mark(now, (uint8_t)(s_src.edges && ph < CYC_PER_SEC / 39U), ph);
            }
        } else if (len >= VS_MIN_CYC) {
            if (len > L->maxCur) L->maxCur = len;
            if (!byIdle && len >= L->maxPrev / 2U) out_mark(start, L->refOk, L->refPh);   /* that level was the vsync */
        }
    }
    L->start = now;
    L->high = high;
    L->valid = 1U;
    L->refOk = 0U;
    if (s_src.edges) {
        const uint32_t ph = now - s_src.last;
        if (ph < CYC_PER_SEC / 39U) { L->refPh = ph; L->refOk = 1U; }
    }
}

static void VSW_IrqCallback(void)
{
    const uint32_t now = DWT->CYCCNT;
    const uint16_t st = VSW_TMRA->STFLR;
    if (st & TMRA_STFLR_CMPF6) {                                    /* output, PB4, either edge */
        VSW_TMRA->STFLR = (uint16_t)(VSW_TMRA->STFLR & ~TMRA_STFLR_CMPF6);
        out_edge(&s_lvl, now, (CM_GPIO->PIDRB & GPIO_PIN_04) ? 1U : 0U);
    }
    if (st & TMRA_STFLR_CMPF7) {                                    /* source, PB5, falling edge */
        VSW_TMRA->STFLR = (uint16_t)(VSW_TMRA->STFLR & ~TMRA_STFLR_CMPF7);
        chan_edge(&s_src, now);
    }
}

void VSW_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;               /* the cycle counter (USB init enables it too) */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_src.pair = 1U;

    /* Registers directly: LL_TMRA is off in this build (hc32f4xx_conf.h) */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_05, GPIO_FUNC_5);         /* TIMA_6_PWM7 */
    SET_REG16_BIT(CM_GPIO->PCRB5, GPIO_PCR_PUU);
    CM_GPIO->PSPCR &= (uint16_t)~GPIO_PSPCR_SPFE_4;              /* PB4: NJTRST released */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_04, GPIO_FUNC_5);         /* TIMA_6_PWM6 */
    SET_REG16_BIT(CM_GPIO->PCRB4, GPIO_PCR_PUU);
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_6, ENABLE);
    VSW_TMRA->PERAR = 0xFFFFU;
    VSW_TMRA->BCSTRL = 0U;                                       /* sawtooth, up, PCLK1, stopped */
    /* source: falling edge, input filter at clock / 16 (a long wire); output: both edges, filter clock / 1 (480p's
     * ~2.5 us line pulses must register, else lines merge into vsync-long levels) */
    VSW_TMRA->CCONR7 = TMRA_CCONR_CAPMD | TMRA_CCONR_HICP1 | TMRA_CCONR_NOFIENCP | (uint16_t)(2U << 13);
    VSW_TMRA->CCONR6 = TMRA_CCONR_CAPMD | TMRA_CCONR_HICP0 | TMRA_CCONR_HICP1 | TMRA_CCONR_NOFIENCP;
    VSW_TMRA->STFLR = 0U;
    VSW_TMRA->ICONR |= TMRA_ICONR_ITEN7 | TMRA_ICONR_ITEN6;

    stc_irq_signin_config_t irq;
    irq.enIRQn = VSW_IRQn;
    irq.enIntSrc = INT_SRC_TMRA_6_CMP;
    irq.pfnCallback = &VSW_IrqCallback;
    if (INTC_IrqSignIn(&irq) != LL_OK) return;
    NVIC_SetPriority(VSW_IRQn, DDL_IRQ_PRIO_04);
    NVIC_ClearPendingIRQ(VSW_IRQn);
    NVIC_EnableIRQ(VSW_IRQn);
    VSW_TMRA->BCSTRL |= TMRA_BCSTRL_START;
    s_on = 1U;
}

static uint32_t mean_ns(const vsw_chan_t *c, uint32_t now)
{
    if (!c->n || now - c->perAt >= PRESENT_CYC) return 0U;      /* only while current: never another mode's */
    uint64_t sum = 0U;
    const uint8_t n = c->n;
    for (uint8_t i = 0U; i < n; i++) sum += c->ring[(uint8_t)((c->w + RING - 1U - i) % RING)];
    return (uint32_t)((sum * 5U + n / 2U) / n);                  /* 200 MHz cycles -> ns */
}

void VSW_Get(AdvVsync *v)
{
    v->flags = 0U; v->seq = 0U; v->srcPeriodNs = 0U; v->outPeriodNs = 0U; v->phaseNs = 0U; v->ageMs = 0U;
    if (!s_on) return;
    const uint32_t t = DWT->CYCCNT;
    __disable_irq();
    const uint8_t src = (uint8_t)(s_src.edges && t - s_src.last < PRESENT_CYC);
    const uint8_t out = (uint8_t)(s_out.edges && t - s_out.last < PRESENT_CYC);
    v->srcPeriodNs = mean_ns(&s_src, t);
    v->outPeriodNs = mean_ns(&s_out, t);
    const uint32_t ph = s_phase, phAt = s_phaseAt;
    const uint8_t phOk = s_phaseOk;
    v->seq = s_seq;
    __enable_irq();
    v->flags = (uint8_t)((src ? ADV_VS_SRC : 0U) | (out ? ADV_VS_OUT : 0U));
    if (src && out && phOk && v->srcPeriodNs) {
        v->flags |= ADV_VS_PHASE;
        v->phaseNs = ph * 5U;
        const uint32_t age = (t - phAt) / (CYC_PER_SEC / 1000U);
        v->ageMs = (uint16_t)(age > 0xFFFFU ? 0xFFFFU : age);
    }
}
