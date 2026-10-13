#include "limits.h"
#include "board.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void limits_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);

    /*
     * NC switch to GND + internal pull-up:
     *   0 = circuit closed / clear
     *   1 = switch pressed OR wire open
     * This is intentionally fail-safe for a broken switch wire.
     */
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    LIMIT_BOTTOM_PIN | LIMIT_TOP_PIN);
}

bool bottom_limit_pressed(void)
{
    return gpio_get(LIMIT_BOTTOM_PORT, LIMIT_BOTTOM_PIN) != 0;
}

bool top_limit_pressed(void)
{
    return gpio_get(LIMIT_TOP_PORT, LIMIT_TOP_PIN) != 0;
}
