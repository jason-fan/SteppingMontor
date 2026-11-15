/**
 * adc.c - STM8S103F3 ADC1 Driver Implementation
 *
 * Single-conversion mode: init selects channel + SPSEL,
 * read starts one conversion and returns the 10-bit result.
 */

#include <stdint.h>
#include "adc.h"

/* ========== GPIO Register Access ========== */

#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* STM8S103 GPIO port base addresses */
#define GPIO_PC                 0x500A
#define GPIO_PD                 0x500F

/* ========== ADC Register Bit Masks ========== */

#define ADC_CSR_EOC             (uint8_t)0x80
#define ADC_CR1_ADON            (uint8_t)0x01
#define ADC_CR1_CONT            (uint8_t)0x02
#define ADC_CR1_SPSEL           (uint8_t)0x70
#define ADC_CR2_ALIGN           (uint8_t)0x08   /* bit3: 0 = left-aligned, 1 = right-aligned */

/* ========== ADC1 Peripheral Clock Gate ==========
 * STM8S103: ADC1 clock is gated by CLK_PCKENR2 bit3 (ADC1EN, 0x08).
 * Without PCKENR2 bit3, every ADC1 register write is ignored, ADON never
 * starts a conversion, EOC never sets -> adc_read() would spin forever. */
#define CLK_PCKENR2             (*(volatile uint8_t *)0x50C8)
#define CLK_PCKENR2_ADC1EN      (uint8_t)0x08

/* EOC poll timeout: prevents adc_read() from hanging the main loop
 * forever if the conversion never completes (clock gate closed / pin
 * misconfigured). ~5000 polls is far beyond a normal 2-8us conversion. */
#define ADC_EOC_TIMEOUT         5000U

/* ========== Static Helpers ========== */

/* Resolve AIN channel -> GPIO port base address */
static uint16_t adc_ch_port(uint8_t ch)
{
    switch (ch) {
        case ADC_CH_PC4:  return GPIO_PC;   /* AIN2 = PC4 */
        case ADC_CH_PD2:  return GPIO_PD;   /* AIN3 = PD2 */
        case ADC_CH_PD3:  return GPIO_PD;   /* AIN4 = PD3 */
        case ADC_CH_PD5:  return GPIO_PD;   /* AIN5 = PD5 */
        case ADC_CH_PD6:  return GPIO_PD;   /* AIN6 = PD6 */
        default:           return 0;        /* invalid channel */
    }
}

/* Resolve AIN channel -> GPIO pin number */
static uint8_t adc_ch_pin(uint8_t ch)
{
    switch (ch) {
        case ADC_CH_PC4:  return 4;
        case ADC_CH_PD2:  return 2;
        case ADC_CH_PD3:  return 3;
        case ADC_CH_PD5:  return 5;
        case ADC_CH_PD6:  return 6;
        default:           return 0;
    }
}

/* ========== Method Implementations ========== */

static void adc_init(ADC_t *self)
{
    uint16_t port = adc_ch_port(self->ch);
    uint8_t  mask = (uint8_t)(1 << adc_ch_pin(self->ch));

    if (port == 0) {
        return;   /* invalid channel, nothing to configure */
    }

    /* Enable ADC1 peripheral clock FIRST (PCKENR2 bit3).
     * Without this, all ADC register writes below are silently ignored. */
    CLK_PCKENR2 |= CLK_PCKENR2_ADC1EN;

    /* GPIO -> input floating: DDR=0, CR1=0, CR2=0 */
    GPIO_DDR(port) &= (uint8_t)~mask;
    GPIO_CR1(port) &= (uint8_t)~mask;
    GPIO_CR2(port) &= (uint8_t)~mask;

    /* CSR: select channel, preserve status bits (incl. EOC) */
    ADC1_CSR = (ADC1_CSR & (uint8_t)0xF0) | (self->ch & 0x0F);

    /* CR2: force ALIGN = 0 (LEFT-aligned) explicitly instead of relying on
     * the reset value. Left alignment puts DATA[9:2] in DRH[7:0] and
     * DATA[1:0] in DRL[7:6], which is exactly what adc_convert_once()
     * assumes when it merges the two bytes. Do NOT set ALIGN = 1 here:
     * right alignment would move the low bits into DRL[1:0] and the
     * merge formula below would return a value 4x too small. */
    ADC1_CR2 &= (uint8_t)~ADC_CR2_ALIGN;

    /* CR1: set SPSEL (mask 0x70), clear CONT (single conversion) */
    ADC1_CR1 = (ADC1_CR1 & (uint8_t)~ADC_CR1_SPSEL)
             | ((self->spsel << 4) & ADC_CR1_SPSEL);
    ADC1_CR1 &= (uint8_t)~ADC_CR1_CONT;

    /* Power up ADC now (ADON 0->1, no conversion triggered).
     * Without this the FIRST adc_read() only powers up on its ADON
     * write and the first EOC poll can time out (sentinel 0xFFFF). */
    ADC1_CR1 |= ADC_CR1_ADON;

    /* No channel has been sampled yet -> the next adc_read() MUST run the
     * settle delay once to charge the sample capacitor. */
    self->last_ch = 0xFF;
}

/* ========== Single Conversion (internal) ==========
 * STM8S ADC1 (unlike STM32) starts a new single-mode conversion on a
 * simple WRITE of 1 to ADON — no 0->1 edge required. Keep ADON high
 * (ADC stays powered) between reads: clearing ADON first would power
 * the ADC down and back up on every sample, and the re-powered
 * conversion often never sets EOC -> timeout -> constant 0V readings.
 * Returns ADC_RAW_ERROR sentinel if EOC never sets within the timeout. */
static uint16_t adc_convert_once(void)
{
    uint16_t guard = 0;
    uint8_t  drh, drl;

    /* Clear EOC before starting a new conversion */
    ADC1_CSR &= (uint8_t)~ADC_CSR_EOC;
    /* Write 1 to ADON: start a new conversion (ADC stays powered) */
    ADC1_CR1 |= ADC_CR1_ADON;
    /* Wait for conversion complete (EOC), with timeout so a broken ADC
     * clock / channel can never hang the main loop. */
    while ((ADC1_CSR & ADC_CSR_EOC) == 0) {
        if (++guard >= ADC_EOC_TIMEOUT) {
            return ADC_RAW_ERROR;   /* EOC never set: propagate the fault */
        }
    }
    /* Merge 10-bit result. Alignment is LEFT (CR2 ALIGN = 0, forced in
     * adc_init), so the data sits in the high bits of the 16-bit pair:
     *   DRH[7:0] = DATA[9:2]
     *   DRL[7:6] = DATA[1:0]   (DRL[5:0] unused)
     * => result = (DRH << 2) | (DRL >> 6), range 0..1023.
     * Read the bytes in TWO statements: the STM8S ADC latches DRL when DRH
     * is read, so DRH must come first, and a single expression gives no
     * evaluation-order guarantee for the two `|` operands. */
    drh = ADC1_DRH;                 /* read first: latches DRL */
    drl = ADC1_DRL;
    return (uint16_t)(((uint16_t)drh << 2) | ((uint16_t)drl >> 6));
}

/* ========== Settling Delay ==========
 * STM8 ADC1 samples its input for a FIXED 1 ADC clock cycle - there is
 * no programmable sampling time. Before starting a conversion we pause
 * briefly so the external input network (potentiometer wiper, wiring,
 * stray capacitance) has time to charge the ADC sample capacitor.
 * NOTE: called from adc_read() ONLY when the channel changed (see
 * last_ch) - charging is a one-off cost per channel selection, not a
 * per-read cost. */
static void adc_settle_delay(void)
{
    volatile uint16_t i;
    for (i = 0; i < 3000; i++) {
        __asm__("nop");
    }
}

static uint16_t adc_read(ADC_t *self)
{
    uint32_t sum = 0;
    uint16_t v;
    uint8_t  i, n = (self->samples < 1) ? 1 : self->samples;

    /* Settle ONLY when the channel changed since the previous read.
     * The 1 ADC clock sample window is fixed and very short, so the sample
     * capacitor needs a pause to charge through the external input network
     * - but ONLY after the channel is (re)selected. Repeating that 3000-nop
     * delay on every read of the SAME channel buys nothing: no charge is
     * lost between two conversions of one channel, the capacitor simply
     * stays at the last sampled potential. */
    if (self->ch != self->last_ch) {
        adc_settle_delay();
        self->last_ch = self->ch;
    }

    /* Average n consecutive conversions; a single-shot reading on an
     * unstable input (pot wiper / wiring) jumps between quantization
     * levels, averaging restores a stable value close to true DC. */
    for (i = 0; i < n; i++) {
        v = adc_convert_once();
        if (v == ADC_RAW_ERROR) {
            /* Do NOT fold the failure into 0x0000 (that reported "0 V", i.e.
             * a pot turned fully CCW, and hid a dead ADC behind a plausible
             * reading). Abort the whole read and hand the sentinel up: it is
             * outside 0..1023, so no caller can confuse it with a real value,
             * and the samples already accumulated are discarded instead of
             * being averaged into a wrong result. */
            return ADC_RAW_ERROR;
        }
        sum += v;
    }
    return (uint16_t)((sum + (n >> 1)) / n);   /* rounded average */
}

static void adc_set_samples(ADC_t *self, uint8_t n)
{
    self->samples = (n < 1) ? 1 : n;
}

static void adc_power_down(ADC_t *self)
{
    (void)self;
    /* ADON = 0: power down ADC */
    ADC1_CR1 &= (uint8_t)~ADC_CR1_ADON;
}

/* ========== Constructor ========== */

ADC_t ADC_Create(uint8_t ch, uint8_t spsel)
{
    ADC_t adc;

    adc.ch        = ch;
    adc.spsel     = spsel;
    adc.samples   = ADC_DEFAULT_SAMPLES;   /* 8 = settle delay + averaging */
    /* 0xFF = "nothing sampled yet", so the first read() always settles.
     * Must be set here too: adc_init() only runs when the caller invokes
     * init(), whereas read() may be called on a Create()-only object. */
    adc.last_ch   = 0xFF;
    adc.init      = adc_init;
    adc.read      = adc_read;
    adc.setSamples= adc_set_samples;
    adc.powerDown = adc_power_down;

    return adc;
}
