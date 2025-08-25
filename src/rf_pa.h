/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#ifndef RF_PA_H
#define RF_PA_H
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  RF_PA_PWR_OFF = 0,   // ~0 mW (PA disabled)
  RF_PA_PWR_20mW,      // ~20 mW
  RF_PA_PWR_100mW,     // ~100 mW
  RF_PA_PWR_200mW,     // ~200 mW
  RF_PA_PWR_800mW,     // ~800 mW
  RF_PA_PWR_COUNT
} rf_pa_power_t;

void rf_pa_init(void);
void rf_pa_enable(bool on);
uint16_t rf_pa_read_vdet_mv(void);
uint16_t rf_pa_get_vref_mv(void);
void rf_pa_set_vref_mv(uint16_t mv);
uint16_t rf_pa_set_power_level(rf_pa_power_t level);

#endif //RF_PA_H
