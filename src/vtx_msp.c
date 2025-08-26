/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#include "vtx_msp.h"
#include "main.h"
#include "msp.h"
#include "msp_displayport.h"
#include "msp_protocol.h"
#include "rf_pa.h"
#include "rtc6705.h"
#include "uart.h"
#include "usb.h"

#include <string.h>
#include <stdio.h>

/* ---- VTX bands table: letter + 8-char name + 8 channel freqs (MHz) ---- */
#define VTX_CHANNEL_COUNT    8
#define VTX_CH_LABEL_COUNT   8
#define VTX_IS_FACTORY_BAND  1

typedef struct {
    char letter;                            /* 'A','B','E','F','R' */
    uint8_t band_name[VTX_CH_LABEL_COUNT];  /* shown in BF “Name”, exactly 8 bytes */
    uint16_t freq[VTX_CHANNEL_COUNT];       /* ch1..ch8, MHz */
} vtx_band_t;

/* Power levels table (index -> mW). Tune for your HW */
static const uint16_t g_power_mw[] = { 25, 100, 200, 800 };
#define NUM_PWR (sizeof(g_power_mw)/sizeof(g_power_mw[0]))

/* VTX bands table (letter + 8-char name + 8 channel freqs (MHz)).
 * These are standard bands used in Betaflight and iNav.
 * You can add custom bands here if needed. */
static const vtx_band_t g_bands[] = {
    /* Band A (Boscam A) */
    { 'A', { 'B','O','S','C','A','M',' ',' ' },
      { 5865,5845,5825,5805,5785,5765,5745,5725 } },

    /* Band B (Boscam B) */
    { 'B', { 'B','O','S','C','A','M',' ',' ' },
      { 5733,5752,5771,5790,5809,5828,5847,5866 } },

    /* Band E */
    { 'E', { 'B','A','N','D',' ',' ',' ',' ' },
      { 5705,5685,5665,5645,5885,5905,5925,5945 } },

    /* Band F (FatShark) */
    { 'F', { 'F','a','t','S','h','a','r','k' },
      { 5740,5760,5780,5800,5820,5840,5860,5880 } },

    /* Band R (Raceband) */
    { 'R', { 'R','a','c','e','b','a','n','d' },
      { 5658,5695,5732,5769,5806,5843,5880,5917 } },
};
#define NUM_BANDS (sizeof(g_bands)/sizeof(g_bands[0]))

static vtx_config_t g_cfg = {
    .band = 5,
    .channel = 1,
    .frequency = 5658,
    .power = 1,
    .pitmode = 0,
};

/* ------------------------- MSP payload definitions -------------------------- */
/* Structures match Betaflight MSP_VTX_CONFIG payload layout (compact). */
#pragma pack(push,1)
typedef struct {
    uint8_t device_type;   /* 0=NONE/unknown, keep 0 */
    uint8_t band;          /* 1..5 or 0 if using frequency */
    uint8_t channel;       /* 1..8 */
    uint8_t power;         /* power index (0..N-1) */
    uint16_t frequency;    /* MHz, non-zero overrides band/channel */
    uint8_t pitmode;       /* 0/1 */
} msp_vtx_config_t;

typedef struct {
    uint8_t band;          /* 1..5 */
    uint8_t channel;       /* 1..8 */
    uint16_t frequency;    /* MHz */
} msp_vtx_table_freq_t;

typedef struct {
    uint8_t index;         /* 0..N-1 */
    uint16_t power_mw;     /* mW */
    char label[16];        /* "25", "100", ... */
} msp_vtx_table_power_t;
#pragma pack(pop)

const vtx_config_t* vtx_get_config(void)
{
    return &g_cfg;
}

static inline void msp_tx_send(uint8_t owner, const uint8_t *buf, uint16_t len)
{
    if (owner == MSP_OWNER_USB) {
        usb_uart_write_bytes((const char*)buf, len);
    } else if (owner == MSP_OWNER_UART) {
        uart1_tx_dma((uint8_t*)buf, len);
    }
}

static inline bool freq_is_in_58ghz(uint16_t mhz)
{
    return (mhz >= 5600 && mhz <= 6000);
}

/* -------------------- Hardware apply: RTC6705 + rf_pa ----------------------- */
/* NOTE:
 * - RTC6705 power has coarse steps (3/7/11/13 dBm). For external PA you
 *   currently drive only VREF (enable) and use detector for telemetry.
 * - Here we only set frequency and pick a coarse internal PA level from power index.
 */
static rtc6705_power_t map_power_to_rtc6705(uint8_t powerIndex)
{
    /* Simple mapping: 25mW→7dBm, 100mW→11dBm, 200/800mW→13dBm (coarse) */
    if (powerIndex == 0) return RTC6705_PA_3dBm;
    if (powerIndex == 1) return RTC6705_PA_7dBm;
    if (powerIndex == 2) return RTC6705_PA_11dBm;
    if (powerIndex == 3) return RTC6705_PA_13dBm;
    return RTC6705_PA_13dBm;
}

static void vtx_apply_hw(const vtx_config_t *cfg)
{
    printf("vtx_apply_hw: band=%d ch=%d freq=%d power=%d pit=%d\r\n",
           cfg->band, cfg->channel, cfg->frequency, cfg->power, cfg->pitmode);

    /* Program synthesizer frequency (MHz) */
    if (freq_is_in_58ghz(cfg->frequency)) {
        rtc6705_set_frequency(cfg->frequency);
    }

    /* External PA enable/pitmode */
    if (cfg->pitmode) {
        /* Set internal RTC6705 PA power */
        rtc6705_allow_power_writes(true);
        rtc6705_set_power(RTC6705_PA_3dBm);
        rtc6705_allow_power_writes(false);

        /* Pit: minimal radiation — disable external RF Power Amplifier */
        rf_pa_set_power_level(RF_PA_PWR_OFF);
    } else {
        /* Set internal RTC6705 PA power */
        rtc6705_allow_power_writes(true);
        rtc6705_set_power(map_power_to_rtc6705(cfg->power));
        rtc6705_allow_power_writes(false);

        /* Set external RF Power Amplifier */
        rf_pa_set_power_level(cfg->power+1);
    }
}

static void handle_msp_set_vtx_config(uint8_t owner, const uint8_t *payload, uint16_t data_size)
{
    /* MSP_VTX_CONFIG payload (Betaflight): 15 bytes
       [0] vtxType
       [1] band (1..N)
       [2] channel (1..8)
       [3] power (1..P)  <-- BF is 1-based
       [4] pit mode (0/1)
       [5] freq LSB
       [6] freq MSB      <-- MHz (non-zero overrides band/channel)
       [7] deviceIsReady
       [8] lowPowerDisarm (0/1)
       [9]  pitModeFreq LSB
       [10] pitModeFreq MSB
       [11] vtxTableAvailable (0/1)
       [12] bands
       [13] channels
       [14] powerLevels
    */

    if (!payload || data_size < 15) {
        return; // malformed
    }
    (void)owner;

    /* Parse all raw fields */
    const uint8_t vtx_type          = payload[0];
    uint8_t band_raw                = payload[1];
    uint8_t ch_raw                  = payload[2];
    uint8_t power_1based            = payload[3];
    uint8_t pitmode                 = payload[4];
    const uint16_t freq_mhz         = (uint16_t)payload[5] | ((uint16_t)payload[6] << 8);
    const uint8_t device_ready      = payload[7];
    const uint8_t low_power_disarm  = payload[8];
    uint16_t pit_mode_freq          = (uint16_t)payload[9] | ((uint16_t)payload[10] << 8);
    uint8_t vtx_table_available     = payload[11];
    uint8_t vtx_table_bands         = payload[12];
    uint8_t vtx_table_channels      = payload[13];
    uint8_t vtx_table_power_levels  = payload[14];

    if (!vtx_table_available) {
        return; // ignore if no VTX table
    }

    /* Normalize power: Betaflight uses 1-based power indices. */
    int power_idx = (power_1based > 0) ? ((int)power_1based - 1) : 0;

    /* If LPD is active, force the lowest power level. */
    if (low_power_disarm) {
        power_idx = 0;
    }

    /* Clamp power index to our table */
    if (power_idx < 0) power_idx = 0;
    if ((unsigned)power_idx >= NUM_PWR) power_idx = (int)NUM_PWR - 1;

    /* Update runtime config */
    g_cfg.pitmode = pitmode ? 1 : 0;
    g_cfg.power = (uint8_t)power_idx;

    g_cfg.frequency = freq_mhz;
    g_cfg.channel = ch_raw;
    g_cfg.band = band_raw;
    g_cfg.vtx_table_available = vtx_table_available;

    /* Apply to hardware */
    static uint16_t last_freq;
    static uint8_t last_power;
    static uint8_t last_pitmode;
    if (last_freq != g_cfg.frequency || last_power != g_cfg.power || last_pitmode != g_cfg.pitmode) {
        vtx_apply_hw(&g_cfg);
        last_freq = g_cfg.frequency;
        last_power = g_cfg.power;
        last_pitmode = g_cfg.pitmode;
    }

#if 0 // debug print
    printf("MSP_VTX_CONFIG parsed: type=%u band=%u ch=%u power=%u pit=%u freq=%u avail=%u\r\n",
           vtxType,
           g_cfg.band,
           g_cfg.channel,
           g_cfg.power,
           g_cfg.pitmode,
           g_cfg.frequency,
           vtx_table_available);
#endif
    // Silence unused variable warnings
    (void) vtx_type;
    (void) device_ready;
    (void) pit_mode_freq;
    (void) vtx_table_bands;
    (void) vtx_table_channels;
    (void) vtx_table_power_levels;
}

/* Small sender wrapper */
static inline void msp_tx_send_owner(uint8_t owner, const uint8_t *buf, uint16_t len)
{
    if (owner == MSP_OWNER_USB) {
        usb_uart_write_bytes((const char*)buf, len);
    } else if (owner == MSP_OWNER_UART) {
        uart1_tx_dma((uint8_t*)buf, len);
    }
}

void vtx_msp_clear_table_and_set_defaults(uint8_t owner)
{
    if (g_cfg.vtx_table_available == 1) {
        return; // VTX table already present, do nothing
    }

    // Reset VTX table to defaults
    uint8_t p[15] = {0};
    p[0]  = 0;                          /* idx LSB (legacy BF field, keep 0) */
    p[1]  = 0;                          /* idx MSB */
    p[2]  = (uint8_t)(NUM_PWR);         /* power index */
    p[3]  = 0;                          /* pitmode (0/1) */
    p[4]  = 0;                          /* lowPowerDisarm */
    p[5]  = 0; p[6]  = 0;               /* pitModeFreq (LSB/MSB), 0 if unused */
    p[7]  = (uint8_t)(NUM_BANDS);       /* newBand (1..NUM_BANDS) */
    p[8]  = VTX_CHANNEL_COUNT;          /* newChannel (1..8) */
    p[9]  = 0; p[10] = 0;               /* newFreq LSB/MSB, 0 => use band/channel */
    p[11] = (uint8_t)(NUM_BANDS);       /* newBandCount: BF expects "6"*/
    p[12] = VTX_CHANNEL_COUNT;          /* newChannelCount (8) */
    p[13] = (uint8_t)(NUM_PWR);         /* newPowerCount: */
    p[14] = 1;                          /* vtx table should be cleared */

    uint8_t tx_buff[64];
    const uint16_t len = construct_msp_command_v1(tx_buff, MSP_SET_VTX_CONFIG, p, sizeof(p), MSP_OUTBOUND);
    msp_tx_send_owner(owner, tx_buff, len);

    // Push new full VTX tables && save to FC EEPROM
    vtx_msp_push_power_table(owner);
    vtx_msp_push_band_table(owner);
    vtx_msp_eeprom_write(owner);
}

/* Power table
 * Send MSP_SET_VTXTABLE_POWERLEVEL for each entry.
 * Payload:
 *   [0] index (1..N)
 *   [1..2] power_mW (uint16 LE)
 *   [3] label_len
 *   [4..] ASCII label (e.g. "25","100","800")
 */
void vtx_msp_push_power_table(uint8_t owner)
{
    for (uint8_t i = 0; i < NUM_PWR; i++) {
        const uint8_t idx1 = (uint8_t)(i + 1);
        const uint16_t mw  = g_power_mw[i];

        char label[16] = {0};
        uint8_t label_len = (uint8_t)snprintf(label, sizeof(label), "%u", (unsigned)mw);
        if (label_len > 15) label_len = 15;

        uint8_t p[1 + 2 + 1 + 16] = {0};
        p[0] = idx1;
        p[1] = (uint8_t)(mw & 0xFF);
        p[2] = (uint8_t)((mw >> 8) & 0xFF);
        p[3] = label_len;
        memcpy(&p[4], label, label_len);

        uint8_t tx_buff[64];
        const uint16_t len = construct_msp_command_v1(tx_buff,
                            MSP_SET_VTXTABLE_POWERLEVEL,
                            p, (uint8_t)(4 + label_len),
                            MSP_OUTBOUND);

        msp_tx_send_owner(owner, tx_buff, len);
    }
}

void vtx_msp_push_band_table(uint8_t owner)
{
    for (uint8_t b = 1; b <= NUM_BANDS; b++) {
        const vtx_band_t *band = &g_bands[b-1];

        /* Payload layout (29 bytes):
           [0]=band(1..N), [1]=nameLen(=8), [2..9]=name8,
           [10]=letter, [11]=isFactory(1), [12]=channels(8),
           [13..28]=8×freq LE16
        */
        uint8_t p[29] = {0};
        p[0] = b;
        p[1] = VTX_CH_LABEL_COUNT;          /* 8 */

        /* Name (exactly 8 bytes) */
        for (uint8_t i = 0; i < VTX_CH_LABEL_COUNT; i++) {
            p[2 + i] = band->band_name[i];
        }

        p[10] = (uint8_t)band->letter;      /* single ASCII letter */
        p[11] = 1;                          /* factory band flag */
        p[12] = VTX_CHANNEL_COUNT;          /* 8 */

        /* 8 frequencies, little-endian MHz */
        for (uint8_t ch = 0; ch < VTX_CHANNEL_COUNT; ch++) {
            const uint16_t f = band->freq[ch];
            p[13 + ch*2 + 0] = (uint8_t)(f & 0xFF);
            p[13 + ch*2 + 1] = (uint8_t)(f >> 8);
        }

        uint8_t tx_buff[64];
        const uint16_t len = construct_msp_command_v2(tx_buff,MSP_SET_VTXTABLE_BAND, p, (uint8_t)sizeof(p), MSP_PACKET_COMMAND);

        msp_tx_send_owner(owner, tx_buff, len);
    }
}

void vtx_msp_eeprom_write(uint8_t owner)
{
    uint8_t tx_buff[64];
    const uint16_t len = construct_msp_command_v1(tx_buff, MSP_EEPROM_WRITE, NULL, 0, MSP_OUTBOUND);
    msp_tx_send_owner(owner, tx_buff, len);
}

void vtx_msp_request_config(uint8_t owner)
{
    uint8_t tx_buff[64];
    const uint16_t len = construct_msp_command_v1(tx_buff, MSP_VTX_CONFIG, NULL, 0, MSP_OUTBOUND);
    msp_tx_send_owner(owner, tx_buff, len);
}

bool vtx_msp_handle_msp(uint8_t owner, uint16_t msp_cmd, uint16_t data_size, const uint8_t *payload)
{
    switch (msp_cmd) {
    case MSP_VTX_CONFIG:
        handle_msp_set_vtx_config(owner, payload, data_size);
        return true;

    case MSP_SET_VTX_CONFIG:
    case MSP_VTXTABLE_BAND:
    case MSP_VTXTABLE_POWERLEVEL:
    default:
        return false;
    }
}
