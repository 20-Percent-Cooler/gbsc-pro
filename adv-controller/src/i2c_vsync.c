/* The vsync wires' readings over I2C, for boards with the J17 mod (the ESP's I2C bus wired to the ADV Controller's
 * I2C2: PA4 SCL, PA5 SDA). A read-only slave at ADV_VSYNC_I2C_ADDR: every read returns the latest frame
 * (advVsyncFramePack), anything written is ignored. A board without the mod never sees a transfer: the pins stay
 * idle inputs and the ESP's probe of the address gets no acknowledge, so it uses the debug pin as before.
 *
 * The frame is built in the main loop (VSW_Get, every I2CV_PERIOD_MS) into one of two buffers and published by
 * flipping an index; the interrupt copies the published one at the start of a read, so a read never mixes two
 * frames. The interrupts only move bytes (no clock stretching beyond a few us). */
#include "i2c_vsync.h"
#include "vsync_wires.h"
#include "i2c_probe.h"
#include "main.h"
#include <string.h>

#define I2CV_UNIT      (CM_I2C2)
#define I2CV_FCG       (FCG1_PERIPH_I2C2)
#define I2CV_SCL_PORT  (GPIO_PORT_A)
#define I2CV_SCL_PIN   (GPIO_PIN_04)
#define I2CV_SCL_FUNC  (GPIO_FUNC_51)
#define I2CV_SDA_PORT  (GPIO_PORT_A)
#define I2CV_SDA_PIN   (GPIO_PIN_05)
#define I2CV_SDA_FUNC  (GPIO_FUNC_50)
#define I2CV_RX_IRQn   (INT008_IRQn)      /* INT003 link UART errors, INT006 vsync wires, INT007 system tick */
#define I2CV_TX_IRQn   (INT009_IRQn)
#define I2CV_EE_IRQn   (INT010_IRQn)
#define I2CV_PERIOD_MS (4U)

static uint8_t s_frame[2][ADV_VSYNC_FRAME_BYTES];
static volatile uint8_t s_pub;            /* the frame a read starts from */
static uint8_t s_tx[ADV_VSYNC_FRAME_BYTES];
static volatile uint8_t s_txi;            /* next byte of the current read; 0 = a read starts */
static volatile uint32_t s_reads;         /* reads started (console / tests) */
static uint32_t s_lastMs;
static uint8_t s_on;
static uint32_t s_stuckSince;            /* ms + 1 when SDA was first seen held low with SCL idle; 0 = not */
static uint32_t s_unsticks;              /* peripheral restarts by the stuck-bus check */
#define I2CV_STUCK_MS  (20U)

static void I2CV_RxIrq(void)               /* a written byte: not used */
{
    (void)I2C_ReadData(I2CV_UNIT);
}

static void I2CV_TxIrq(void)
{
    if (s_txi == 0U) {
        memcpy(s_tx, s_frame[s_pub], sizeof(s_tx));
        s_reads++;
    }
    I2C_WriteData(I2CV_UNIT, s_txi < sizeof(s_tx) ? s_tx[s_txi] : 0xFFU);
    if (s_txi < sizeof(s_tx)) s_txi++;
}

static void I2CV_EventIrq(void)            /* STOP or NACK (the master's last byte): the next read starts over */
{
    const uint32_t st = I2CV_UNIT->SR & (I2C_FLAG_STOP | I2C_FLAG_NACKF);
    s_txi = 0U;
    if (st != 0U) I2C_ClearStatus(I2CV_UNIT, st);
}

static int32_t signIn(IRQn_Type irqn, en_int_src_t src, func_ptr_t cb)
{
    stc_irq_signin_config_t irq;
    irq.enIRQn = irqn;
    irq.enIntSrc = src;
    irq.pfnCallback = cb;
    const int32_t r = INTC_IrqSignIn(&irq);
    if (r != LL_OK) return r;
    NVIC_SetPriority(irqn, DDL_IRQ_PRIO_05);   /* below the vsync capture (04): its timestamps come first */
    NVIC_ClearPendingIRQ(irqn);
    NVIC_EnableIRQ(irqn);
    return LL_OK;
}

static int32_t periphInit(void)
{
    I2C_DeInit(I2CV_UNIT);
    stc_i2c_init_t cfg;
    float32_t err;
    (void)I2C_StructInit(&cfg);
    cfg.u32ClockDiv = I2C_CLK_DIV4;
    cfg.u32Baudrate = 400000UL;   /* the slave follows the master's clock; this sets the filters' timing */
    cfg.u32SclTime = 3UL;
    if (I2C_Init(I2CV_UNIT, &cfg, &err) != LL_OK) return LL_ERR;
    I2C_SlaveAddrConfig(I2CV_UNIT, I2C_ADDR0, I2C_ADDR_7BIT, ADV_VSYNC_I2C_ADDR);
    I2C_IntCmd(I2CV_UNIT, I2C_INT_RX_FULL | I2C_INT_TX_EMPTY | I2C_INT_STOP | I2C_INT_NACK, ENABLE);
    I2C_Cmd(I2CV_UNIT, ENABLE);
    return LL_OK;
}

void I2CV_Init(void)
{
    AdvVsync v;
    VSW_Get(&v);
    advVsyncFramePack(&v, s_frame[0]);
    memcpy(s_frame[1], s_frame[0], ADV_VSYNC_FRAME_BYTES);

    GPIO_SetFunc(I2CV_SCL_PORT, I2CV_SCL_PIN, I2CV_SCL_FUNC);
    GPIO_SetFunc(I2CV_SDA_PORT, I2CV_SDA_PIN, I2CV_SDA_FUNC);
    FCG_Fcg1PeriphClockCmd(I2CV_FCG, ENABLE);
    if (signIn(I2CV_RX_IRQn, INT_SRC_I2C2_RXI, &I2CV_RxIrq) != LL_OK ||
        signIn(I2CV_TX_IRQn, INT_SRC_I2C2_TXI, &I2CV_TxIrq) != LL_OK ||
        signIn(I2CV_EE_IRQn, INT_SRC_I2C2_EEI, &I2CV_EventIrq) != LL_OK)
        return;
    if (periphInit() != LL_OK) return;
    s_on = 1U;
}

/* Standard I2C bus recovery from this side: SCL clocked (open drain) until the slave holding SDA lets go, then a
 * STOP. Only while nobody else drives the bus (SDA held low with SCL idle for I2CV_STUCK_MS: no transfer does that,
 * so the ESP is in reset or being flashed). */
static uint8_t busRecover(uint8_t *released)   /* returns the clocks it took */
{
    stc_gpio_init_t g;
    (void)GPIO_StructInit(&g);
    g.u16PinDir = PIN_DIR_OUT;
    g.u16PinOutputType = PIN_OUT_TYPE_NMOS;   /* open drain: never drives a 1 */
    g.u16PinState = PIN_STAT_SET;
    GPIO_SetFunc(I2CV_SCL_PORT, I2CV_SCL_PIN, GPIO_FUNC_0);
    GPIO_SetFunc(I2CV_SDA_PORT, I2CV_SDA_PIN, GPIO_FUNC_0);
    (void)GPIO_Init(I2CV_SCL_PORT, I2CV_SCL_PIN, &g);
    (void)GPIO_Init(I2CV_SDA_PORT, I2CV_SDA_PIN, &g);
    DDL_DelayUS(5U);
    uint8_t clocks = 0U;
    for (; clocks < 9U && (CM_GPIO->PIDRA & I2CV_SDA_PIN) == 0U; clocks++) {
        GPIO_ResetPins(I2CV_SCL_PORT, I2CV_SCL_PIN); DDL_DelayUS(5U);
        GPIO_SetPins(I2CV_SCL_PORT, I2CV_SCL_PIN);   DDL_DelayUS(5U);
    }
    *released = (CM_GPIO->PIDRA & I2CV_SDA_PIN) != 0U ? 1U : 0U;
    GPIO_ResetPins(I2CV_SCL_PORT, I2CV_SCL_PIN); DDL_DelayUS(5U);   /* STOP: SDA rises while SCL is high */
    GPIO_ResetPins(I2CV_SDA_PORT, I2CV_SDA_PIN); DDL_DelayUS(5U);
    GPIO_SetPins(I2CV_SCL_PORT, I2CV_SCL_PIN);   DDL_DelayUS(5U);
    GPIO_SetPins(I2CV_SDA_PORT, I2CV_SDA_PIN);   DDL_DelayUS(5U);
    g.u16PinDir = PIN_DIR_IN;
    (void)GPIO_Init(I2CV_SCL_PORT, I2CV_SCL_PIN, &g);
    (void)GPIO_Init(I2CV_SDA_PORT, I2CV_SDA_PIN, &g);
    GPIO_SetFunc(I2CV_SCL_PORT, I2CV_SCL_PIN, I2CV_SCL_FUNC);
    GPIO_SetFunc(I2CV_SDA_PORT, I2CV_SDA_PIN, I2CV_SDA_FUNC);
    return clocks;
}

/* A master reset in the middle of a transfer (the ESP flashed or reset by esptool) can leave a slave on the ESP's bus
 * driving a 0 bit with SCL released: SDA stays low and every chip disappears, so the ESP takes the board for a
 * standard one (its boot-time recovery doesn't always free it). No transfer holds SDA low with SCL high for long
 * (only START/STOP edges): after I2CV_STUCK_MS this slave restarts and clocks the bus free. */
static void unstick(uint32_t nowMs)
{
    const uint32_t in = CM_GPIO->PIDRA;
    const uint8_t stuck = ((in & I2CV_SDA_PIN) == 0U) && ((in & I2CV_SCL_PIN) != 0U);
    if (!stuck) { s_stuckSince = 0U; return; }
    if (s_stuckSince == 0U) { s_stuckSince = nowMs + 1U; return; }
    const uint32_t stuckMs = nowMs + 1U - s_stuckSince;
    if (stuckMs < I2CV_STUCK_MS) return;
    s_stuckSince = 0U;
    s_unsticks++;
    I2C_Cmd(I2CV_UNIT, DISABLE);
    s_txi = 0U;
    uint8_t released = 0U;
    const uint8_t clocks = busRecover(&released);
    (void)periphInit();
    I2CProbe_ReportStuck(stuckMs, clocks, released);
}

void I2CV_Task(uint32_t nowMs)
{
    if (!s_on) return;
    unstick(nowMs);
    if (nowMs - s_lastMs < I2CV_PERIOD_MS) return;
    s_lastMs = nowMs;
    AdvVsync v;
    VSW_Get(&v);
    const uint8_t next = (uint8_t)(s_pub ^ 1U);   /* the ISR only reads s_frame[s_pub] */
    advVsyncFramePack(&v, s_frame[next]);
    __DMB();
    s_pub = next;
}

uint32_t I2CV_Reads(void) { return s_reads; }
uint8_t I2CV_Running(void) { return s_on; }
uint32_t I2CV_Unsticks(void) { return s_unsticks; }
