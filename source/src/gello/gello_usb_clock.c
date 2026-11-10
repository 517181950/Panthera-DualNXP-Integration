/* Zephyr's FRDM board hook currently enables USB clocks only for device
 * drivers. Supply the corresponding host clock setup without altering the
 * 150 MHz robot PLL or the 48 MHz UART clock. J11 VBUS is managed by the
 * onboard PTN5150A / NX5P3090 CC circuit (UM12018 figure 10). */
#include <zephyr/init.h>
#include <fsl_clock.h>
#include <soc.h>
#include <errno.h>
static int gello_usb_clock_init(void)
{
    if(!(SCG0->SOSCCSR&SCG_SOSCCSR_SOSCVLD_MASK))return -ENODEV;
    if(!(SCG0->LDOCSR&SCG_LDOCSR_LDOEN_MASK)) {
        SCG0->TRIM_LOCK=0x5a5a0001U;
        SCG0->LDOCSR|=SCG_LDOCSR_LDOEN_MASK;
    }
    unsigned budget=1500000;
    while(!(SCG0->LDOCSR&SCG_LDOCSR_VOUT_OK_MASK) && --budget) {}
    if(!budget)return -ETIMEDOUT;
    SYSCON->CLOCK_CTRL|=SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK |
        SYSCON_CLOCK_CTRL_CLKIN_ENA_FM_USBH_LPT_MASK;
    CLOCK_EnableClock(kCLOCK_UsbHs);
    CLOCK_EnableClock(kCLOCK_UsbHsPhy);
    if(!CLOCK_EnableUsbhsPhyPllClock(kCLOCK_Usbphy480M,24000000U))return -EIO;
    CLOCK_EnableUsbhsClock();
    return 0;
}
SYS_INIT(gello_usb_clock_init,PRE_KERNEL_1,1);
