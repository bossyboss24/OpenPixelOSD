/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#ifndef VTX_MSP_H
#define VTX_MSP_H
#include <stdint.h>
#include <stdbool.h>

/* Optional helpers to query current state (for OSD, logs, etc.) */
typedef struct {
  uint8_t band;        // 1..5 (A/B/E/F/R), 0 if using frequency
  uint8_t channel;     // 1..8
  uint16_t frequency;  // MHz; if nonzero, overrides band/channel on SET
  uint8_t power;       // power index (0..N-1)
  uint8_t pitmode;     // 0/1
  uint8_t vtx_table_available;
} vtx_config_t;

const vtx_config_t* vtx_get_config(void);
bool vtx_msp_handle_msp(uint8_t owner, uint16_t msp_cmd, uint16_t data_size, const uint8_t *payload);
void vtx_msp_request_config(uint8_t owner);

void vtx_msp_clear_table_and_set_defaults(uint8_t owner);
void vtx_msp_push_power_table(uint8_t owner);
void vtx_msp_push_band_table(uint8_t owner);
void vtx_msp_eeprom_write(uint8_t owner);

#endif //VTX_MSP_H
