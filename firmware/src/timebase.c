#include "timebase.h"

#include "manipulator.h"
#include "platform.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

static volatile uint32_t g_millis;

void clock_init(void)
{
    /*
     * Standard STM32F401 Black Pill boards use a 25 MHz HSE crystal.
     * This preset configures an 84 MHz system clock and an exact 48 MHz
     * USB clock through PLLQ, which is preferable to HSI for USB FS.
     * If your board uses a different HSE, change this one line.
     */
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
}

void timebase_init(void)
{
    g_millis = 0;

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(84000U - 1U); /* 84 MHz / 84,000 = 1 kHz */
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();
}

uint32_t millis(void)
{
    return g_millis;
}

uint32_t micros(void)
{
    /*
     * SysTick counts down from 83,999 at 84 MHz and generates the 1 kHz
     * millisecond tick.  Read the millisecond counter around CVR so a tick
     * interrupt cannot produce a torn timestamp.  This 32-bit monotonic
     * timestamp wraps naturally after about 71.6 minutes; consumers should
     * compute deltas with unsigned subtraction.
     */
    const volatile uint32_t *const systick_current =
        (const volatile uint32_t *)0xE000E018UL;

    uint32_t before;
    uint32_t after;
    uint32_t current;
    do {
        before = g_millis;
        current = *systick_current & 0x00FFFFFFU;
        after = g_millis;
    } while (before != after);

    const uint32_t elapsed_ticks = (84000U - 1U) - current;
    return before * 1000U + elapsed_ticks / 84U;
}

void sys_tick_handler(void)
{
    ++g_millis;
}

void fast_tick_init(void)
{
    /*
     * TIM4 clock is 84 MHz because APB1 is divided by 2 and STM32 timers get
     * a x2 clock multiplier when the APB prescaler is not 1.
     *
     * 84 MHz / 84 = 1 MHz timer counter
     * 1 MHz / 50 = 20 kHz update interrupt
     */
    rcc_periph_clock_enable(RCC_TIM4);
    rcc_periph_reset_pulse(RST_TIM4);
    timer_set_prescaler(TIM4, 84U - 1U);
    timer_set_period(TIM4, 50U - 1U);
    timer_generate_event(TIM4, TIM_EGR_UG);
    timer_clear_flag(TIM4, TIM_SR_UIF);
    timer_enable_irq(TIM4, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_enable_counter(TIM4);
}

void tim4_isr(void)
{
    if (!timer_get_flag(TIM4, TIM_SR_UIF)) {
        return;
    }

    timer_clear_flag(TIM4, TIM_SR_UIF);

    platform_fast_tick();
    manipulator_fast_tick();
}
