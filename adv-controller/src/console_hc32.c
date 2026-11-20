/* Device glue of the console (HC32 DDL): USART3 port (non-blocking ring), USB CDC port (usb_console.c), I2C access */
#include "i2c_vsync.h"
#include "i2c_probe.h"
#include "vsync_wires.h"
#include <stdio.h>
#include "console_hc32.h"
#include "console.h"
#include "debug_uart.h"
#include "main.h"
#include "link.h"
#include "link_hal.h"
#include "console_fmt.h"
#include "usb_console.h"

#define I2C_TIMEOUT TIMEOUT   /* DDL polling tries: 1000 was shorter than one byte at 200 kHz (3be1653) */

static int uartConnected(void) { return 1; }
static const ConsolePort kUart = {DebugUart_Write, uartConnected, DebugUart_TxSpace};

static int i2cRead(uint8_t dev, uint8_t reg, uint8_t *val) { return I2C_Receive(dev, &reg, val, 1, I2C_TIMEOUT) == 0 ? 0 : -1; }
static int i2cWrite(uint8_t dev, uint8_t reg, uint8_t val)
{
    uint8_t cmd[2] = {reg, val};
    return I2C_Transmit(dev, cmd, 2, I2C_TIMEOUT) == 0 ? 0 : -1;
}

static void cmdInfo(uint8_t port, const char *args)
{
    (void)args;
    AdvInfo a;
    halInfo(&a);
    ConInfo i = {a.fw, a.buildId, a.uptimeS, LINK_RxFrames(), LINK_BadFrames(), LINK_DroppedReplies(),
                 LINK_AppliedSeq(), LINK_StateHash(), LINK_Input(), (a.flags & ADV_INFO_LOCK) ? 1U : 0U,
                 USBCON_Up(), USBCON_IrqRate(), USBCON_IrqMaxUs(), USBCON_Dropped()};
    char b[320];
    conFmtInfo(&i, b, sizeof(b));
    CONSOLE_Write(port, b);
}

static void cmdVsync(uint8_t port, const char *args)   /* vsync wires and their I2C slave (J17 mod) */
{
    (void)args;
    AdvVsync v;
    VSW_Get(&v);
    char b[160];
    snprintf(b, sizeof(b), "vsync: flags %u src %lu ns out %lu ns phase %lu ns | i2c 0x%02X %s, reads %lu, unsticks %lu\r\n",
             (unsigned)v.flags, (unsigned long)v.srcPeriodNs, (unsigned long)v.outPeriodNs, (unsigned long)v.phaseNs,
             (unsigned)ADV_VSYNC_I2C_ADDR, I2CV_Running() ? "on" : "off", (unsigned long)I2CV_Reads(),
             (unsigned long)I2CV_Unsticks());
    CONSOLE_Write(port, b);
}

/* Passive probe of the ESP's I2C bus (i2c_probe.h): p1 changes, p2 raw writes, p0 off, ps stats, pc forget,
 * ph read histogram, po1/po0 OLED transfers */
static void cmdP1(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_Enable(PROBE_MODE_CHANGES); }
static void cmdP2(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_Enable(PROBE_MODE_RAW); }
static void cmdP0(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_Enable(PROBE_MODE_OFF); }
static void cmdPs(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_PrintStats(); }
static void cmdPc(uint8_t port, const char *a) { (void)a; I2CProbe_Forget(); CONSOLE_Write(port, "[PROBE] register copies cleared\r\n"); }
static void cmdPh(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_PrintHistogram(); }
static void cmdPo1(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_OledLog(1U); }
static void cmdPo0(uint8_t port, const char *a) { (void)port; (void)a; I2CProbe_OledLog(0U); }

static void cmdUpdate(uint8_t port, const char *args)
{
    (void)args;
    ConUpd u;
    u.state = LINK_UpdateState();
    u.err = LINK_UpdateError();
    LINK_UpdateProgress(&u.next, &u.size, &u.erased, &u.sectors);
    char b[96];
    conFmtUpdate(&u, b, sizeof(b));
    CONSOLE_Write(port, b);
}

void CONSOLE_HC32_Init(const ConsolePort *usb)
{
    CONSOLE_Init(&kUart, usb, i2cRead, i2cWrite);
    CONSOLE_AddCommand("info", cmdInfo);
    CONSOLE_AddCommand("vs", cmdVsync);
    CONSOLE_AddCommand("p1", cmdP1);
    CONSOLE_AddCommand("p2", cmdP2);
    CONSOLE_AddCommand("p0", cmdP0);
    CONSOLE_AddCommand("ps", cmdPs);
    CONSOLE_AddCommand("pc", cmdPc);
    CONSOLE_AddCommand("ph", cmdPh);
    CONSOLE_AddCommand("po1", cmdPo1);
    CONSOLE_AddCommand("po0", cmdPo0);
    CONSOLE_AddCommand("update", cmdUpdate);
    CONSOLE_Banner(0);
    CONSOLE_Write(0, ">");
}

void CONSOLE_HC32_Poll(uint32_t nowMs)
{
    DebugUart_ClearErrors();
    char ch;
    for (uint8_t k = 0; k < 16U && DebugUart_ReceiveChar(&ch) == 0; k++) CONSOLE_Feed(0, ch);
    LinkEvent e;                             /* link events for "log on" ports, a few per pass */
    for (uint8_t k = 0; k < 4U && LINK_PopEvent(&e); k++) {
        char b[48];
        conFmtEvent(&e, b, sizeof(b));
        CONSOLE_Log("%s", b);
    }
    CONSOLE_Poll(nowMs);
    DebugUart_Poll();
}
