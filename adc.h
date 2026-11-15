/**
 * adc.h - STM8S103F3 ADC1 Driver (Object-Oriented)
 *
 * STM8S103F3 (low-density) TSSOP20 ADC channels:
 *   AIN2 = PC4, AIN3 = PD2, AIN4 = PD3, AIN5 = PD5, AIN6 = PD6
 *   (PA1 is NOT an ADC input on STM8S103F3)
 *
 * Usage:
 *   ADC_t adc = ADC_Create(ADC_CH_PC4, 6);  // potentiometer on PC4
 *   adc.init(&adc);
 *   uint16_t v = adc.read(&adc);           // 0..1023, or ADC_RAW_ERROR
 *   adc.powerDown(&adc);
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* ========== ADC1 Register Addresses (base 0x5400) ========== */
#define ADC1_CSR    (*(volatile uint8_t *)0x5400)  /* bit3:0 CH sel, bit7 EOC */
#define ADC1_CR1    (*(volatile uint8_t *)0x5401)  /* bit0 ADON, bit1 CONT, bit6:4 SPSEL */
#define ADC1_CR2    (*(volatile uint8_t *)0x5402)
#define ADC1_DRH    (*(volatile uint8_t *)0x5404)  /* result high byte */
#define ADC1_DRL    (*(volatile uint8_t *)0x5405)  /* result low byte */

/* ========== Channel Mapping (AIN# -> pin) ========== */
#define ADC_CH_PC4  2   /* AIN2 -> PC4 */
#define ADC_CH_PD2  3   /* AIN3 -> PD2 */
#define ADC_CH_PD3  4   /* AIN4 -> PD3 */
#define ADC_CH_PD5  5   /* AIN5 -> PD5 */
#define ADC_CH_PD6  6   /* AIN6 -> PD6 */

/* ========== ADC Class Definition ========== */

typedef struct ADC ADC_t;

struct ADC {
    uint8_t ch;          /* AIN channel number (2..6) */
    uint8_t spsel;       /* SPSEL sampling clock divider (0..7): 0=/2, 1=/3,
                          * 2=/4, 3=/6, 4=/8, 5=/10, 6=/12, 7=/18 (NOT a
                          * power-of-two ladder). Keep fADC in 1..4MHz, so
                          * with fMASTER=16MHz use 6 (/12 = 1.333MHz). */
    uint8_t samples;     /* averaging filter: samples per read() (1 = off) */
    uint8_t last_ch;     /* channel the sample capacitor was last charged for.
                          * adc_read() runs the settle delay ONLY when
                          * self->ch != self->last_ch (i.e. right after init,
                          * or after a future channel switch). 0xFF = "no
                          * channel sampled yet", set by adc_init() and
                          * ADC_Create() so the very first read still settles.
                          * The settle delay exists to let the external input
                          * network charge the ADC sample capacitor after the
                          * channel changes - repeating it on every read of the
                          * same channel buys nothing and wastes ~187us. */

    /* --- Public methods --- */
    void     (*init)(ADC_t *self);
    uint16_t (*read)(ADC_t *self);      /* filtered: average of samples */
    void     (*setSamples)(ADC_t *self, uint8_t n); /* configure filter */
    void     (*powerDown)(ADC_t *self);
};

/* ========== Constructor ========== */

/**
 * Create an ADC object bound to the given AIN channel.
 * Uses static storage — safe for single-instance use on STM8S103.
 *
 * @param ch     AIN channel number (2..6), use ADC_CH_* macros
 * @param spsel  SPSEL value (0..7): 0=/2, 1=/3, 2=/4, 3=/6, 4=/8, 5=/10,
 *               6=/12, 7=/18. Use 6 for fMASTER=16MHz (fADC = 1.333MHz,
 *               inside the 1..4MHz spec).
 * @return       fully initialized ADC_t object
 */
ADC_t ADC_Create(uint8_t ch, uint8_t spsel);

/* Default averaging filter depth used by ADC_Create (8 samples/read).
 * adc_read() runs the short settling delay ONLY when the channel changed
 * since the previous read (see last_ch above), then averages 8 consecutive
 * conversions for a stable result.
 * Set to 1 via adc.setSamples(&adc, 1) for single shot. */
#define ADC_DEFAULT_SAMPLES  8

/* ========== Error Sentinel ==========
 * adc.read() returns EITHER a valid 10-bit conversion result (0..1023) OR
 * this sentinel, which means "no valid result": the EOC poll timed out
 * (ADC clock gate closed, channel/pin misconfigured, sample capacitor never
 * charging = broken wiper / open wiring).
 * CONTRACT: any return value in 0..1023 is a real reading; every value
 * outside that range (in practice exactly ADC_RAW_ERROR) is an error and
 * MUST NOT be treated as 0 V. The old behaviour folded the timeout into
 * 0x0000, which disguised a hardware fault as "pot turned fully
 * counter-clockwise" and silently drove the motor to its slowest step with
 * no serial warning. Callers must branch on ADC_RAW_ERROR instead. */
#define ADC_RAW_ERROR        0xFFFFU

#endif /* ADC_H */
