#include "encoders.h"

#include "board.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/*
 * x4 quadrature decoder.
 *
 * Each valid A/B edge contributes +/-1 count. Invalid two-bit jumps contribute
 * zero; those normally indicate noise or transitions that arrived faster than
 * the interrupt service path could observe them.
 */
static const int8_t QUADRATURE_DELTA[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

typedef struct {
    uint32_t a_port;
    uint16_t a_pin;
    uint32_t b_port;
    uint16_t b_pin;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

static encoder_t g_encoders[4] = {
    { ENC1_C1_PORT, ENC1_C1_PIN, ENC1_C2_PORT, ENC1_C2_PIN, 0, 0 },
    { ENC2_C1_PORT, ENC2_C1_PIN, ENC2_C2_PORT, ENC2_C2_PIN, 0, 0 },
    { ENC3_C1_PORT, ENC3_C1_PIN, ENC3_C2_PORT, ENC3_C2_PIN, 0, 0 },
    { ENC4_C1_PORT, ENC4_C1_PIN, ENC4_C2_PORT, ENC4_C2_PIN, 0, 0 },
};

static uint8_t read_state(const encoder_t *encoder)
{
    const uint8_t a = gpio_get(encoder->a_port, encoder->a_pin) ? 1U : 0U;
    const uint8_t b = gpio_get(encoder->b_port, encoder->b_pin) ? 1U : 0U;
    return (uint8_t)((a << 1U) | b);
}

static void update_encoder(unsigned index)
{
    encoder_t *const encoder = &g_encoders[index];
    const uint8_t current = read_state(encoder);
    const uint8_t transition = (uint8_t)((encoder->previous_state << 2U) | current);

    encoder->count += QUADRATURE_DELTA[transition & 0x0FU];
    encoder->previous_state = current;
}

static void configure_exti(uint32_t line, uint32_t port)
{
    exti_select_source(line, port);
    exti_set_trigger(line, EXTI_TRIGGER_BOTH);
    exti_reset_request(line);
    exti_enable_request(line);
}

void encoders_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SYSCFG);

    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    ENC1_C1_PIN | ENC1_C2_PIN | ENC2_C1_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    ENC2_C2_PIN | ENC3_C1_PIN | ENC3_C2_PIN |
                    ENC4_C1_PIN | ENC4_C2_PIN);

    for (unsigned i = 0; i < 4U; ++i) {
        g_encoders[i].count = 0;
        g_encoders[i].previous_state = read_state(&g_encoders[i]);
    }

    /* Every signal is on a unique EXTI line, so all eight channels can be
     * decoded on both edges without line-sharing conflicts. */
    configure_exti(EXTI8, GPIOA);   /* FR A: PA8 */
    configure_exti(EXTI9, GPIOA);   /* FR B: PA9 */
    configure_exti(EXTI15, GPIOA);  /* RR A: PA15 */
    configure_exti(EXTI3, GPIOB);   /* RR B: PB3 */
    configure_exti(EXTI4, GPIOB);   /* RL A: PB4 */
    configure_exti(EXTI5, GPIOB);   /* RL B: PB5 */
    configure_exti(EXTI6, GPIOB);   /* FL A: PB6 */
    configure_exti(EXTI7, GPIOB);   /* FL B: PB7 */

    nvic_enable_irq(NVIC_EXTI3_IRQ);
    nvic_enable_irq(NVIC_EXTI4_IRQ);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
}

void encoders_get_counts(encoder_counts_t *counts)
{
    if (counts == 0) {
        return;
    }

    counts->front_right = g_encoders[0].count;
    counts->rear_right  = g_encoders[1].count;
    counts->rear_left   = g_encoders[2].count;
    counts->front_left  = g_encoders[3].count;
}

void encoders_reset(void)
{
    for (unsigned i = 0; i < 4U; ++i) {
        g_encoders[i].count = 0;
        g_encoders[i].previous_state = read_state(&g_encoders[i]);
    }
}

void exti3_isr(void)
{
    if (exti_get_flag_status(EXTI3)) {
        exti_reset_request(EXTI3);
        update_encoder(1);
    }
}

void exti4_isr(void)
{
    if (exti_get_flag_status(EXTI4)) {
        exti_reset_request(EXTI4);
        update_encoder(2);
    }
}

void exti9_5_isr(void)
{
    if (exti_get_flag_status(EXTI5)) {
        exti_reset_request(EXTI5);
        update_encoder(2);
    }
    if (exti_get_flag_status(EXTI6)) {
        exti_reset_request(EXTI6);
        update_encoder(3);
    }
    if (exti_get_flag_status(EXTI7)) {
        exti_reset_request(EXTI7);
        update_encoder(3);
    }
    if (exti_get_flag_status(EXTI8)) {
        exti_reset_request(EXTI8);
        update_encoder(0);
    }
    if (exti_get_flag_status(EXTI9)) {
        exti_reset_request(EXTI9);
        update_encoder(0);
    }
}

void exti15_10_isr(void)
{
    if (exti_get_flag_status(EXTI15)) {
        exti_reset_request(EXTI15);
        update_encoder(1);
    }
}
