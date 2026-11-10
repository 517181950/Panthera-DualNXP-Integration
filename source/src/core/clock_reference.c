/* Keep the nominal 150 MHz core/CAN clock, but derive PLL0 from the board's
 * 24 MHz crystal instead of the internal 48 MHz FRO. The board early hook
 * already starts this crystal. Run before SysTick's PRE_KERNEL_2 init. */
#include <zephyr/init.h>
#include <fsl_clock.h>
#include <soc.h>
#include <errno.h>

static int robot_clock_reference_init(void)
{
    if ((SCG0->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) == 0U) {
        return -ENODEV;
    }
    const pll_setup_t crystal_pll = {
        .pllctrl = SCG_APLLCTRL_SOURCE(0U) | SCG_APLLCTRL_SELI(27U) |
                   SCG_APLLCTRL_SELP(13U),
        .pllndiv = SCG_APLLNDIV_NDIV(4U),
        .pllpdiv = SCG_APLLPDIV_PDIV(1U),
        .pllmdiv = SCG_APLLMDIV_MDIV(50U),
        .pllRate = 150000000U,
    };
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);
    pll_error_t result = CLOCK_SetPLL0Freq(&crystal_pll);
    CLOCK_AttachClk(kPLL0_to_MAIN_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U);
    SystemCoreClock = 150000000U;
    return result == kStatus_PLL_Success ? 0 : -EIO;
}

SYS_INIT(robot_clock_reference_init, PRE_KERNEL_1, 0);
