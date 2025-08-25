/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#include "rf_pa.h"
#include "main.h"
#include <stdbool.h>

/* --- Calibration table: DAC VREF in mV for each target output power --- */
/* NOTE: placeholder values, adjust experimentally on real HW */
static const uint16_t g_pa_vref_table[RF_PA_PWR_COUNT] = {
    [RF_PA_PWR_OFF]   = 0,     // PA off
    [RF_PA_PWR_20mW]  = 1800,  // TODO: calibrate
    [RF_PA_PWR_100mW] = 1800,  // TODO: calibrate
    [RF_PA_PWR_200mW] = 1800,  // TODO: calibrate
    [RF_PA_PWR_800mW] = 1800   // TODO: calibrate
};

static uint16_t g_vref_mv = 0;

static inline void dac_ch2_write_mv(uint16_t mv)
{
    uint32_t dac_raw = DAC12BIT_FROM_MV(mv);
    if (dac_raw > 4095u) dac_raw = 4095u;
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, dac_raw);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);

    g_vref_mv = mv;
}

void rf_pa_init(void)
{
    /* Enable DAC1 ch2 if not already enabled by user init */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
    rf_pa_enable(false); // keep PA off at boot
}

void rf_pa_enable(bool on)
{
    dac_ch2_write_mv(on ? g_vref_mv : 0u);
}

void rf_pa_set_vref_mv(uint16_t mv)
{
    dac_ch2_write_mv(mv);
}

uint16_t rf_pa_get_vref_mv(void)
{
    return g_vref_mv;
}

uint16_t rf_pa_read_vdet_mv(void)
{
    return adc_read_mv(ADC_CH_PA_VDET);
}

uint16_t rf_pa_set_power_level(rf_pa_power_t level)
{
    // TODO: add adjustment PA Vdet vs actual output power
    if (level >= RF_PA_PWR_COUNT) {
        level = RF_PA_PWR_OFF;
    }
    uint16_t mv = g_pa_vref_table[level];
    rf_pa_set_vref_mv(mv);
    return mv;
}

