#pragma once

#include <stdint.h>

#include "bsp/def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_LCD_WIDTH 240U
#define BSP_LCD_HEIGHT 280U

typedef uint16_t bsp_lcd_color_t;

#define BSP_LCD_RGB(red, green, blue) ((bsp_lcd_color_t) ( \
    (((uint32_t) (red) & 0xF8U) << 8U) | \
    (((uint32_t) (green) & 0xFCU) << 3U) | \
    (((uint32_t) (blue) & 0xF8U) >> 3U) \
))

typedef enum {
    BSP_LCD_ASCII_FONT_CLASSIC = 0,
    BSP_LCD_ASCII_FONT_BLOCK,
    BSP_LCD_ASCII_FONT_COUNT,
} bsp_lcd_ascii_font_t;

typedef enum {
    BSP_LCD_KEY_MIDDLE = 0,
    BSP_LCD_KEY_UP,
    BSP_LCD_KEY_DOWN,
    BSP_LCD_KEY_LEFT,
    BSP_LCD_KEY_RIGHT,
    BSP_LCD_KEY_COUNT,
    BSP_LCD_KEY_NONE = BSP_LCD_KEY_COUNT,
} bsp_lcd_key_t;

typedef enum {
    BSP_LCD_KEY_CLICK = 0,
    BSP_LCD_KEY_DOUBLE_CLICK,
    BSP_LCD_KEY_LONG_PRESS,
    BSP_LCD_KEY_EVENT_COUNT,
} bsp_lcd_key_event_t;

typedef void (*bsp_lcd_key_callback_t)(
    bsp_lcd_key_t key,
    bsp_lcd_key_event_t event,
    void *context
);

enum {
    BSP_LCD_COLOR_BLACK = 0x0000,
    BSP_LCD_COLOR_BLUE = 0x001F,
    BSP_LCD_COLOR_GREEN = 0x07E0,
    BSP_LCD_COLOR_CYAN = 0x07FF,
    BSP_LCD_COLOR_RED = 0xF800,
    BSP_LCD_COLOR_MAGENTA = 0xF81F,
    BSP_LCD_COLOR_YELLOW = 0xFFE0,
    BSP_LCD_COLOR_WHITE = 0xFFFF,
};

bsp_status_t bsp_lcd_init(void);
void bsp_lcd_set_backlight(uint8_t enabled);

bsp_status_t bsp_lcd_clear(bsp_lcd_color_t color);
bsp_status_t bsp_lcd_fill(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    bsp_lcd_color_t color
);
bsp_status_t bsp_lcd_draw_pixel(uint16_t x, uint16_t y, bsp_lcd_color_t color);
bsp_status_t bsp_lcd_draw_line(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    bsp_lcd_color_t color
);
bsp_status_t bsp_lcd_draw_rect(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    bsp_lcd_color_t color
);
bsp_status_t bsp_lcd_draw_circle(
    uint16_t center_x,
    uint16_t center_y,
    uint16_t radius,
    bsp_lcd_color_t color
);
bsp_status_t bsp_lcd_draw_ascii_char(
    uint16_t x,
    uint16_t y,
    char character,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    uint8_t scale
);
bsp_status_t bsp_lcd_draw_ascii_char_font(
    uint16_t x,
    uint16_t y,
    char character,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    bsp_lcd_ascii_font_t font,
    uint8_t scale
);
bsp_status_t bsp_lcd_draw_ascii(
    uint16_t x,
    uint16_t y,
    const char *text,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    uint8_t scale
);
bsp_status_t bsp_lcd_draw_ascii_font(
    uint16_t x,
    uint16_t y,
    const char *text,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    bsp_lcd_ascii_font_t font,
    uint8_t scale
);
bsp_status_t bsp_lcd_blit_rgb565(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    const bsp_lcd_color_t *pixels
);

bsp_status_t bsp_lcd_set_key_callback(
    bsp_lcd_key_t key,
    bsp_lcd_key_event_t event,
    bsp_lcd_key_callback_t callback,
    void *context
);

/**
 * 在任务上下文中周期调用，建议调用周期不大于 10 ms
 */
void bsp_lcd_process_keys(void);

#ifdef __cplusplus
}
#endif
