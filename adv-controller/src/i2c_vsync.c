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

void I2CV_Init(void)
{
    AdvVsync v;
    VSW_Get(&v);
    advVsyncFramePack(&v, s_frame[0]);
    memcpy(s_frame[1], s_frame[0], ADV_VSYNC_FRAME_BYTES);

    GPIO_SetFunc(I2CV_SCL_PORT, I2CV_SCL_PIN, I2CV_SCL_FUNC);
    GPIO_SetFunc(I2CV_SDA_PORT, I2CV_SDA_PIN, I2CV_SDA_FUNC);
    FCG_Fcg1PeriphClockCmd(I2CV_FCG, ENABLE);
    I2C_DeInit(I2CV_UNIT);
    stc_i2c_init_t cfg;
    float32_t err;
    (void)I2C_StructInit(&cfg);
    cfg.u32ClockDiv = I2C_CLK_DIV4;
    cfg.u32Baudrate = 400000UL;   /* the slave follows the master's clock; this sets the filters' timing */
    cfg.u32SclTime = 3UL;
    if (I2C_Init(I2CV_UNIT, &cfg, &err) != LL_OK) return;
    I2C_SlaveAddrConfig(I2CV_UNIT, I2C_ADDR0, I2C_ADDR_7BIT, ADV_VSYNC_I2C_ADDR);
    if (signIn(I2CV_RX_IRQn, INT_SRC_I2C2_RXI, &I2CV_RxIrq) != LL_OK ||
        signIn(I2CV_TX_IRQn, INT_SRC_I2C2_TXI, &I2CV_TxIrq) != LL_OK ||
        signIn(I2CV_EE_IRQn, INT_SRC_I2C2_EEI, &I2CV_EventIrq) != LL_OK)
        return;
    I2C_IntCmd(I2CV_UNIT, I2C_INT_RX_FULL | I2C_INT_TX_EMPTY | I2C_INT_STOP | I2C_INT_NACK, ENABLE);
    I2C_Cmd(I2CV_UNIT, ENABLE);
    s_on = 1U;
}

void I2CV_Task(uint32_t nowMs)
{
    if (!s_on || nowMs - s_lastMs < I2CV_PERIOD_MS) return;
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
