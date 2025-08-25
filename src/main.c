/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#include "main.h"
#include "msp_displayport.h"
#include "system.h"
#include "usb.h"
#include "video_gen.h"
#include "video_overlay.h"

#include "rtc6705.h"

#include <rf_pa.h>
#include <stdio.h>

#if defined(HIGH_RAM)
#include "video_graphics.h"
extern bool new_field;
#endif

#define LED_BLINK_INTERVAL 100 // milliseconds
#define DEBUG_LOOP_INTERVAL 100 // milliseconds

void led_blink(void);

void debug_print_loop(void)
{
    static uint32_t last_tick = 0;

    if ((HAL_GetTick() - last_tick) >= DEBUG_LOOP_INTERVAL) {
        last_tick = HAL_GetTick();
        uint16_t vdet = rf_pa_read_vdet_mv();
        printf("PA Vdet: %umV, Set PA Vref: %umV\r\n", vdet, rf_pa_get_vref_mv());
        printf("MCU Temp: %.2fC, Vdd: %lumV\r\n", adc_read_mcu_temp_c(), adc_read_vdda_mv());
    }
}


int main (void)
{
    HAL_Init();
    SystemClock_Config();
    gpio_init();
    usb_init();
    dma_init();
    adc_init();

    video_overlay_init();
#if defined(HIGH_RAM)
    video_graphics_init();
    video_draw_text_system_font(FONT_SYSTEM_WIDTH * 2, VIDEO_HEIGHT - FONT_SYSTEM_HEIGHT, "WAITING MSP...");
    video_graphics_draw_complete();
#endif
    msp_displayport_init();

    if(rtc6705_init()) {
        printf("rtc6705 detected\n\n");
        rtc6705_set_frequency(5880);
        rtc6705_set_power(RTC6705_PA_3dBm);

        rf_pa_init();
        rf_pa_set_power_level(RF_PA_PWR_20mW);
    }

    while (1)
    {
        msp_loop_process();
        led_blink();
        debug_print_loop();

#if 0 // TODO: remove later
// For test only - 3D cube animation
#if defined(HIGH_RAM)
        if (new_field == false) {
            video_draw_3d_cube_animation();
        }
#endif
#endif

    }
}

void led_blink(void)
{
    static uint32_t last_tick = 0;

    if ((HAL_GetTick() - last_tick) >= LED_BLINK_INTERVAL) {
        LED_STATE_GPIO_Port->ODR ^= LED_STATE_Pin;
        last_tick = HAL_GetTick();
    }
}

