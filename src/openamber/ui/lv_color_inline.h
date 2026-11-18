#pragma once
// LVGL declares lv_color_make() out of line and it returns a 3-byte struct. In the generated setup() every
// call then gets its own stack slot that GCC never reuses (no end-of-life marker on the return temporary):
// ~1700 widget colors cost ~5 kB of loop task stack. As a constant aggregate the color needs no slot.
// Same result as lv_color_make() in lv_color.c (LVGL 9.x: lv_color_t is {blue, green, red}).
// Included from main.cpp after esphome.h, so LVGL (and its own lv_color_make declaration) is already in.

#define lv_color_make(r, g, b) (lv_color_t{(uint8_t) (b), (uint8_t) (g), (uint8_t) (r)})
