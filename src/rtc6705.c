/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#include "rtc6705.h"
#include "main.h"

/** rtc6705.c
 * Bit-banged 3-wire interface for RTC6705
 *      Datasheet notes (RTC6705):
 *      - 25-bit frame, LSB-first: A0..A3 (4b) + RW (1=write,0=read) + D0..D19 (20b).
 *      - FRF = 2 * (N*64 + A) * (Fosc / R). Typical Fosc=8 MHz, R=400 => Fref=20 kHz.
 *      - Registers:
 *      0x00: Synth A (R-counter, 15b). Default R=400.
 *      0x01: Synth B (N[12:0], A[6:0]).
 *      0x02: Synth C (charge pump, test, etc).
 *      0x07: Pre-driver & PA control; PA5G_PW[1:0] controls PA power level.
 *      0x0F: STATE readable register.
 * (Refer to the PDF for details.)
 */

#define SDIO_PORT   SPI2_MOSI_GPIO_Port
#define SDIO_PIN    SPI2_MOSI_Pin
#define SCLK_PORT   SPI2_SCK_GPIO_Port
#define SCLK_PIN    SPI2_SCK_Pin
#define LE_PORT     SPI2_CS_GPIO_Port
#define LE_PIN      SPI2_CS_Pin

#ifndef RTC6705_SDIO_INPUT_PULL
#define RTC6705_SDIO_INPUT_PULL   LL_GPIO_PULL_UP
#endif

// Short delay loop iterations between edges (tune to taste).
#ifndef RTC6705_DELAY_ITERS
#define RTC6705_DELAY_ITERS 8
#endif

// Convenience macros
#define SDIO_OUT()          gpio_to_output(SDIO_PORT, SDIO_PIN)
#define SDIO_IN()           gpio_to_input (SDIO_PORT, SDIO_PIN, RTC6705_SDIO_INPUT_PULL)
#define SDIO_WR(b)          gpio_write(SDIO_PORT, SDIO_PIN, (b))
#define SDIO_RD()           gpio_read(SDIO_PORT, SDIO_PIN)

#define SCLK_OUT()          gpio_to_output(SCLK_PORT, SCLK_PIN)
#define SCLK_WR(b)          gpio_write(SCLK_PORT, SCLK_PIN, (b))

#define LE_OUT()            gpio_to_output(LE_PORT, LE_PIN)
#define LE_WR(b)            gpio_write(LE_PORT, LE_PIN, (b))

// Synth ref source
#ifndef RTC6705_FOSC_HZ
#define RTC6705_FOSC_HZ     8000000u  // 8 MHz crystal per datasheet
#endif
#ifndef RTC6705_R_DIV
#define RTC6705_R_DIV       400u      // R-counter default = 400 (-> 20 kHz ref)
#endif

// Register addresses
#define RTC6705_REG_SYN_A   0x00  // R-counter (15 bits effective)
#define RTC6705_REG_SYN_B   0x01  // N[12:0], A[6:0]
#define RTC6705_REG_SYN_C   0x02  // charge pump, etc
#define RTC6705_REG_VCO3    0x07  // PA control (PA5G_PW[1:0] etc)
#define RTC6705_REG_STATE   0x0F  // readable state

// Bitfield helpers for REG 0x01 (N/A)
#define SYNB_N_MASK         0x1FFFu      // 13 bits
#define SYNB_A_MASK         0x007Fu      // 7 bits (use <=63 for /64 prescaler)
#define SYNB_A_SHIFT        0
#define SYNB_N_SHIFT        7

// Bitfield for REG 0x00 (R 15 bits)
#define SYNA_R_MASK         0x7FFFu

// Bitfield for REG 0x07 (PA5G_PW[1:0] at bits [8:7])
#define REG7_PA5G_PW_SHIFT  7
#define REG7_PA5G_PW_MASK   (0x3u << REG7_PA5G_PW_SHIFT)

static void rtc6705_write_reg(uint8_t addr4, uint32_t data20);
static uint32_t rtc6705_read_reg(uint8_t addr4);
static bool rtc6705_detect(void);

static inline void bb_delay(void)
{
    // ~tens of ns per a few NOPs depending on CPU clock; tweak RTC6705_DELAY_ITERS for coarse control
    for (volatile uint32_t i = 0; i < RTC6705_DELAY_ITERS; ++i) {
        __NOP();
    }
}

// Switch given pins to General Purpose Output (MODER=01)
static inline void gpio_to_output(GPIO_TypeDef *port, uint32_t pinmask)
{
    for (uint32_t bit = 0; bit < 16; bit++) {
        if (pinmask & (1u << bit)) {
            uint32_t pos = bit * 2u;
            port->MODER = (port->MODER & ~(0x3u << pos)) | (0x1u << pos);
        }
    }
}

// Switch given pins to Input (MODER=00) and set PUPDR
static inline void gpio_to_input(GPIO_TypeDef *port, uint32_t pinmask, uint32_t pull)
{
    for (uint32_t bit = 0; bit < 16; bit++) {
        if (pinmask & (1u << bit)) {
            uint32_t pos = bit * 2u;
            port->MODER &= ~(0x3u << pos);
        }
    }
    for (uint32_t bit = 0; bit < 16; bit++) {
        if (pinmask & (1u << bit)) {
            uint32_t pos = bit * 2u;
            port->PUPDR = (port->PUPDR & ~(0x3u << pos)) | ((pull & 0x3u) << pos);
        }
    }
}

// Atomic write: set or reset
static inline void gpio_write(GPIO_TypeDef *port, uint32_t pinmask, uint32_t level)
{
    if (level) port->BSRR = pinmask; else port->BRR = pinmask;
}
static inline uint32_t gpio_read(GPIO_TypeDef *port, uint32_t pinmask)
{
    return (port->IDR & pinmask) ? 1u : 0u;
}

// Shift one bit LSB-first on rising SCLK (mode 0 semantics)
static inline void shift_out_bit_lsbf(uint32_t bit)
{
    SDIO_WR(bit & 1u);
    bb_delay();
    SCLK_WR(1);
    bb_delay();
    SCLK_WR(0);
}

// Read one bit (chip drives on falling; we sample on rising to meet mode 0)
static inline uint32_t shift_in_bit_lsbf(void)
{
    bb_delay();
    SCLK_WR(1);
    uint32_t b = SDIO_RD();
    bb_delay();
    SCLK_WR(0);
    return b;
}

// LSB-first 25-bit frame builder: A0..A3, RW, D0..D19
static inline uint32_t make_frame(uint8_t addr4, uint8_t rw, uint32_t data20)
{
    uint32_t f = 0;
    f |= ((addr4 & 0x1u) << 0);
    f |= (((addr4 >> 1) & 0x1u) << 1);
    f |= (((addr4 >> 2) & 0x1u) << 2);
    f |= (((addr4 >> 3) & 0x1u) << 3);
    f |= ((rw & 0x1u) << 4);
    f |= ((data20 & 0xFFFFFu) << 5);
    return f;
}


/**
 * @brief Initialize GPIO clocks and idle levels for RTC6705 bit-bang interface.
 * Call this once before any read/write. Make sure SPI2 peripheral is disabled on these pins.
 */
bool rtc6705_init(void)
{
    SDIO_OUT();
    SCLK_OUT();
    LE_OUT();

    // Idle levels: SCLK=0, LE=0, SDIO=0
    SCLK_WR(0);
    LE_WR(0);
    SDIO_WR(0);
    // Program default R (optional but explicit)
    rtc6705_write_reg(RTC6705_REG_SYN_A, (RTC6705_R_DIV & SYNA_R_MASK));

    return rtc6705_detect();
}

/**
 * @brief Write 20-bit value to 4-bit register address.
 * @param addr4  lower 4 bits used
 * @param data20 lower 20 bits used
 */
static void rtc6705_write_reg(uint8_t addr4, uint32_t data20)
{
    uint32_t frame = make_frame(addr4 & 0xFu, 1u, data20);

    SDIO_OUT();
    LE_WR(0);

    // Shift 25 bits LSB-first
    for (int i = 0; i < 25; i++) {
        shift_out_bit_lsbf(frame >> i);
    }

    // Latch on LE rising edge
    bb_delay();
    LE_WR(1);
    bb_delay();
    LE_WR(0);
}

/**
 * @brief Read 20-bit value from 4-bit register address.
 * @param addr4 lower 4 bits used
 * @return 20-bit data value (LSB-aligned)
 */
static uint32_t rtc6705_read_reg(uint8_t addr4)
{
    uint32_t header = make_frame(addr4 & 0xFu, 0u, 0u);

    SDIO_OUT();
    LE_WR(0);

    // Send only first 5 bits (A0-A3 + RW=0)
    for (int i = 0; i < 5; i++) {
        shift_out_bit_lsbf(header >> i);
    }

    // Now chip drives the bus -> switch SDIO to input
    SDIO_IN();
    bb_delay();

    uint32_t data = 0;
    for (int i = 0; i < 20; i++) {
        data |= (shift_in_bit_lsbf() << i);
    }
    return (data & 0xFFFFFu);
}

/**
 * @brief Set internal PA output power level via PA5G_PW[1:0] (REG 0x07).
 * @param level one of rtc6705_power_t values (3, 7, 11, 13 dBm).
 */
void rtc6705_set_power(rtc6705_power_t level)
{
    uint32_t reg = rtc6705_read_reg(RTC6705_REG_VCO3);
    reg &= ~REG7_PA5G_PW_MASK;
    reg |= ((uint32_t)level << REG7_PA5G_PW_SHIFT) & REG7_PA5G_PW_MASK;
    rtc6705_write_reg(RTC6705_REG_VCO3, reg);
}

/**
 * @brief Compute and program synthesizer for requested frequency.
 * @param freq_mhz desired RF in MHz (e.g., 5865 for 5.865 GHz).
 * @return actually programmed frequency in MHz (quantized to 40 kHz steps).
 *
 * Formula: FRF = 2 * (N*64 + A) * (Fosc / R).
 * With Fosc=8 MHz, R=400 => step = 40 kHz.
 */
uint32_t rtc6705_set_frequency(uint32_t freq_mhz)
{
    const uint32_t fosc = RTC6705_FOSC_HZ;     // 8,000,000
    const uint32_t rdiv = RTC6705_R_DIV;       // 400
    const uint32_t fref_hz = fosc / rdiv;      // 20,000 Hz
    const uint32_t step_hz = 2u * fref_hz;     // 40,000 Hz

    uint64_t target_hz = ((uint64_t)freq_mhz) * 1000000ull;

    // total = (N*64 + A)
    uint64_t total = (target_hz + (step_hz/2ull)) / step_hz; // round
    uint32_t N = (uint32_t)(total / 64ull);
    uint32_t A = (uint32_t)(total % 64ull);
    if (A > 63u) { A = 63u; if (N) N--; }   // clamp A

    uint32_t regB = ((N & SYNB_N_MASK) << SYNB_N_SHIFT) |
                    ((A & SYNB_A_MASK) << SYNB_A_SHIFT);

    rtc6705_write_reg(RTC6705_REG_SYN_A, (RTC6705_R_DIV & SYNA_R_MASK));
    rtc6705_write_reg(RTC6705_REG_SYN_B, regB);

    uint64_t actual_hz = (uint64_t)step_hz * (uint64_t)(N * 64u + A);
    return (uint32_t)(actual_hz / 1000000ull); // return in MHz
}

/**
 * @brief Detect presence of RTC6705 by reading the STATE register (0x0F).
 *
 * Heuristics:
 *  - Read the 20-bit STATE register several times.
 *  - Reject if all reads are stuck at 0x00000 or 0xFFFFF.
 *  - Accept if all reads are identical and non-pathological.
 *  - Otherwise allow variation only in the lower 3 bits (STATE[2:0]),
 *    while upper bits [19:3] must remain stable.
 *
 * @return true if the chip is likely present and responding, false otherwise.
 */
static bool rtc6705_detect(void)
{
    uint32_t s[4];
    for (int i = 0; i < 4; ++i) {
        s[i] = rtc6705_read_reg(RTC6705_REG_STATE) & 0xFFFFF; // mask to 20 bits
    }

    // Reject if all samples are zero or all ones
    bool all_zero = (s[0] == 0) && (s[1] == 0) && (s[2] == 0) && (s[3] == 0);
    bool all_ones = (s[0] == 0xFFFFF) && (s[1] == 0xFFFFF) &&
                    (s[2] == 0xFFFFF) && (s[3] == 0xFFFFF);
    if (all_zero || all_ones)
        return false;

    // Accept if all samples are identical and valid
    bool all_same = (s[0] == s[1]) && (s[1] == s[2]) && (s[2] == s[3]);
    if (all_same)
        return true;

    // Otherwise check if high bits [19:3] are stable across samples
    uint32_t hi_mask = 0xFFFF8; // bits 19..3
    bool hi_stable = ((s[0] & hi_mask) == (s[1] & hi_mask)) &&
                     ((s[1] & hi_mask) == (s[2] & hi_mask)) &&
                     ((s[2] & hi_mask) == (s[3] & hi_mask));

    return hi_stable;
}

/**
 * @brief Smoketest: read the readable STATE register (0x0F).
 *        Returns STATE[2:0] in bits [2:0] of the returned 20-bit value.
 */
uint32_t rtc6705_smoketest(void)
{
    // Ensure interface is initialized before calling.
    // Read STATE (address 0x0F): documented readable register.
    uint32_t state = rtc6705_read_reg(RTC6705_REG_STATE);
    // You can set a breakpoint here to see 'state' (bits [2:0] show current state).
    return state;
}