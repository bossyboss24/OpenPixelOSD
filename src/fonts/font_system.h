#pragma once
#include <stdint.h>

/* 8x12 cell, 1bpp, 64 bytes/glyph, 256 glyphs, MSB-first per row */
#define FONT_SYSTEM_WIDTH         8
#define FONT_SYSTEM_HEIGHT        12
#define FONT_SYSTEM_BPP           1
#define FONT_SYSTEM_BYTES_PER_ROW ((FONT_SYSTEM_WIDTH + 7) / 8)
#define FONT_SYSTEM_STRIDE        64
#define FONT_SYSTEM_COUNT         256

extern const uint8_t font_system[FONT_SYSTEM_COUNT * FONT_SYSTEM_STRIDE];

typedef struct {
    const uint8_t *data;
    uint8_t glyph_w;
    uint8_t glyph_h;
    uint8_t bytes_per_row;
    uint8_t stride;
} font1bpp_t;

static inline const font1bpp_t* font_system_get(void) {
    static const font1bpp_t f = {
        .data = font_system,
        .glyph_w = FONT_SYSTEM_WIDTH,
        .glyph_h = FONT_SYSTEM_HEIGHT,
        .bytes_per_row = FONT_SYSTEM_BYTES_PER_ROW,
        .stride = FONT_SYSTEM_STRIDE
    };
    return &f;
}
