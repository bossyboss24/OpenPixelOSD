#!/usr/bin/env python3
# Generate a crisp 1bpp pixel font for OpenPixelOSD.
# - Produces two files: <prefix>.h (extern + descriptor) and <prefix>.c (definition)
# - 256 glyphs (codes 0..255) using a single 8-bit encoding (default cp1251)
# - Each glyph WxH pixels (default 12x18), 1bpp, MSB-first, row-major
# - Fixed stride of 64 bytes per glyph (padded/truncated)
# - In the .c file each glyph starts with a comment:  /* Hex 0xXX, Char '…' */


from PIL import Image, ImageDraw, ImageFont
import argparse, sys, unicodedata
from pathlib import Path

STRIDE_BYTES = 64  # bytes per glyph

USAGE_EXAMPLE = """
Example usage:
---------------
python3 make_pixel_font_1bpp.py \\
    --ttf python/ttf_fonts/TerminusTTF-4.49.3.ttf \\
    --size 14 \\
    --encoding cp1251 \\
    --width 8 \\
    --height 12 \\
    --threshold 32 \\
    --prefix src/fonts/font_system
"""

class ExampleArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\n")
        self.print_help(sys.stderr)
        sys.stderr.write(USAGE_EXAMPLE + "\n")
        sys.exit(2)

def pack_row_msb_first(bits):
    out = []
    b = 0; cnt = 0
    for bit in bits:
        b = (b << 1) | (1 if bit else 0)
        cnt += 1
        if cnt == 8:
            out.append(b)
            b = 0; cnt = 0
    if cnt:
        b <<= (8 - cnt)
        out.append(b)
    return bytes(out)

def render_glyph(ch, font, W, H, ox, oy, threshold):
    img = Image.new("L", (W, H), 0)
    draw = ImageDraw.Draw(img)
    draw.text((ox, oy), ch, font=font, fill=255)
    img = img.point(lambda v: 255 if v >= threshold else 0, mode='1')
    data = bytearray()
    for y in range(H):
        bits = [1 if img.getpixel((x, y)) else 0 for x in range(W)]
        data += pack_row_msb_first(bits)
    if len(data) < STRIDE_BYTES:
        data += b"\x00" * (STRIDE_BYTES - len(data))
    return data[:STRIDE_BYTES]

def char_for_comment(byte_val, encoding):
    try:
        ch = bytes([byte_val]).decode(encoding, errors="strict")
    except Exception:
        return ""
    if ch == "'": return "\\'"
    if ch == "\\": return "\\\\"
    if unicodedata.category(ch).startswith('C'):
        return ""
    return ch

def write_header(h_path, macro_prefix, array_name, W, H):
    h = f"""#pragma once
#include <stdint.h>

/* {W}x{H} cell, 1bpp, 64 bytes/glyph, 256 glyphs, MSB-first per row */
#define {macro_prefix}_WIDTH         {W}
#define {macro_prefix}_HEIGHT        {H}
#define {macro_prefix}_BPP           1
#define {macro_prefix}_BYTES_PER_ROW (({macro_prefix}_WIDTH + 7) / 8)
#define {macro_prefix}_STRIDE        64
#define {macro_prefix}_COUNT         256

extern const uint8_t {array_name}[{macro_prefix}_COUNT * {macro_prefix}_STRIDE];

typedef struct {{
    const uint8_t *data;
    uint8_t glyph_w;
    uint8_t glyph_h;
    uint8_t bytes_per_row;
    uint8_t stride;
}} font1bpp_t;

static inline const font1bpp_t* {array_name}_get(void) {{
    static const font1bpp_t f = {{
        .data = {array_name},
        .glyph_w = {macro_prefix}_WIDTH,
        .glyph_h = {macro_prefix}_HEIGHT,
        .bytes_per_row = {macro_prefix}_BYTES_PER_ROW,
        .stride = {macro_prefix}_STRIDE
    }};
    return &f;
}}
"""
    h_path.write_text(h, encoding="utf-8")

def write_source(c_path, header_rel, macro_prefix, array_name, glyph_bytes, comments, per_line=16):
    head = f"""#include <stdint.h>
#include "{header_rel}"

#if defined(__GNUC__)
#define FONT_SECTION __attribute__((section(".system_font")))
#else
#define FONT_SECTION
#endif

/* Auto-generated 1bpp font table (flat array).
 * Data is MSB-first row-major, {macro_prefix}_STRIDE bytes per glyph.
 */
const uint8_t {array_name}[{macro_prefix}_COUNT * {macro_prefix}_STRIDE] FONT_SECTION = {{
"""
    body_lines = []
    total_glyphs = len(glyph_bytes)
    for gi in range(total_glyphs):
        # comment for glyph
        body_lines.append(comments[gi])
        data = glyph_bytes[gi]
        for i, b in enumerate(data):
            # Last byte of the whole array must have no trailing comma in strict C89,
            # though C99+ allows trailing commas. We'll omit comma on the very last byte.
            is_last = (gi == total_glyphs - 1) and (i == len(data) - 1)
            sep = "" if is_last else ","
            if (i % per_line) == 0:
                body_lines.append("  ")
            body_lines[-1] += f"0x{b:02X}{sep} "
            if (i % per_line) == (per_line - 1):
                body_lines.append("\n")
        if not body_lines[-1].endswith("\n"):
            body_lines[-1] += "\n"
        body_lines.append("\n")
    tail = "};\n"
    c_path.write_text(head + "".join(body_lines) + tail, encoding="utf-8")

def main():
    ap = ExampleArgumentParser(description="Generate 1bpp pixel font (.h + .c) for OpenPixelOSD")
    ap.add_argument("--ttf", required=True)
    ap.add_argument("--size", type=int, default=14)
    ap.add_argument("--encoding", default="cp1251")
    ap.add_argument("--width", type=int, default=12)
    ap.add_argument("--height", type=int, default=18)
    ap.add_argument("--xoff", type=int, default=0)
    ap.add_argument("--yoff", type=int, default=0)
    ap.add_argument("--threshold", type=int, default=128)
    ap.add_argument("--prefix", required=True)

    # If no args → show help + example
    if len(sys.argv) == 1:
        ap.print_help(sys.stderr)
        sys.stderr.write(USAGE_EXAMPLE + "\n")
        sys.exit(1)

    args = ap.parse_args()

    W, H = args.width, args.height
    try:
        font = ImageFont.truetype(args.ttf, args.size)
    except Exception as e:
        ap.error(f"cannot load font '{args.ttf}': {e}")

    glyph_bytes = []
    comments = []
    for code in range(256):
        label = char_for_comment(code, args.encoding)
        comments.append(f"/* Hex 0x{code:02X}, Char '{label}' */\n")
        try:
            ch = bytes([code]).decode(args.encoding, errors="strict")
        except Exception:
            ch = "\uFFFD"
        data = render_glyph(ch, font, W, H, args.xoff, args.yoff, args.threshold)
        glyph_bytes.append(data)

    prefix_path = Path(args.prefix)
    h_path = prefix_path.with_suffix(".h")
    c_path = prefix_path.with_suffix(".c")
    macro_prefix = prefix_path.stem.upper()
    array_name = prefix_path.stem.lower()

    write_header(h_path, macro_prefix, array_name, W, H)
    write_source(c_path, h_path.name, macro_prefix, array_name, glyph_bytes, comments)

    print(f"Wrote {h_path}")
    print(f"Wrote {c_path}")
    print(f"Total payload: {sum(len(g) for g in glyph_bytes)} bytes")

if __name__ == "__main__":
    main()