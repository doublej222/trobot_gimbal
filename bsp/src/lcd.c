#include "bsp/lcd.h"

#include <stddef.h>
#include <string.h>

#include "bsp/adc.h"
#include "bsp/sys.h"
#include "bsp/time.h"
#include "main.h"
#include "spi.h"

#define LCD_COMMAND_SLEEP_OUT 0x11U
#define LCD_COMMAND_INVERSION_ON 0x21U
#define LCD_COMMAND_DISPLAY_ON 0x29U
#define LCD_COMMAND_COLUMN_ADDRESS 0x2AU
#define LCD_COMMAND_ROW_ADDRESS 0x2BU
#define LCD_COMMAND_MEMORY_WRITE 0x2CU

#define LCD_X_OFFSET 0U
#define LCD_Y_OFFSET 20U
#define LCD_PIXEL_BUFFER_SIZE 128U
#define LCD_SPI_TIMEOUT_MS 10U

#define LCD_ASCII_GLYPH_WIDTH 5U
#define LCD_ASCII_GLYPH_HEIGHT 7U
#define LCD_ASCII_CELL_WIDTH 6U
#define LCD_ASCII_CELL_HEIGHT 8U
#define LCD_ASCII_MAX_SCALE 3U

#define LCD_KEY_DEBOUNCE_MS 10U
#define LCD_KEY_DOUBLE_CLICK_MS 500U
#define LCD_KEY_LONG_PRESS_MS 800U

typedef struct {
    bsp_lcd_key_callback_t callback;
    void *context;
} lcd_key_callback_entry_t;

typedef struct {
    bsp_lcd_key_t candidate_key;
    bsp_lcd_key_t stable_key;
    bsp_lcd_key_t pressed_key;
    bsp_lcd_key_t pending_click_key;
    uint32_t candidate_since;
    uint32_t press_started_at;
    uint32_t pending_click_at;
    uint8_t initialized;
    uint8_t long_press_sent;
    uint8_t second_press;
} lcd_key_tracker_t;

static lcd_key_callback_entry_t key_callbacks[BSP_LCD_KEY_COUNT][BSP_LCD_KEY_EVENT_COUNT];
static lcd_key_tracker_t key_tracker;
static uint8_t lcd_initialized;

static const uint8_t lcd_ascii_digits[][LCD_ASCII_GLYPH_WIDTH] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
};

static const uint8_t lcd_ascii_uppercase[][LCD_ASCII_GLYPH_WIDTH] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
};

static const uint8_t lcd_ascii_lowercase[][LCD_ASCII_GLYPH_WIDTH] = {
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x48, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
};

static const uint8_t lcd_ascii_space[LCD_ASCII_GLYPH_WIDTH] = {0};
static const uint8_t lcd_ascii_question[LCD_ASCII_GLYPH_WIDTH] = {
    0x02, 0x01, 0x51, 0x09, 0x06,
};
static const uint8_t lcd_ascii_greater[LCD_ASCII_GLYPH_WIDTH] = {
    0x00, 0x41, 0x22, 0x14, 0x08,
};
static const uint8_t lcd_ascii_colon[LCD_ASCII_GLYPH_WIDTH] = {
    0x00, 0x36, 0x36, 0x00, 0x00,
};
static const uint8_t lcd_ascii_minus[LCD_ASCII_GLYPH_WIDTH] = {
    0x08, 0x08, 0x08, 0x08, 0x08,
};
static const uint8_t lcd_ascii_period[LCD_ASCII_GLYPH_WIDTH] = {
    0x00, 0x60, 0x60, 0x00, 0x00,
};
static const uint8_t lcd_ascii_slash[LCD_ASCII_GLYPH_WIDTH] = {
    0x20, 0x10, 0x08, 0x04, 0x02,
};

static const uint8_t *lcd_ascii_glyph(char character) {
    if (character >= '0' && character <= '9') {
        return lcd_ascii_digits[(uint8_t) character - (uint8_t) '0'];
    }
    if (character >= 'A' && character <= 'Z') {
        return lcd_ascii_uppercase[(uint8_t) character - (uint8_t) 'A'];
    }
    if (character >= 'a' && character <= 'z') {
        return lcd_ascii_lowercase[(uint8_t) character - (uint8_t) 'a'];
    }
    if (character == ' ') return lcd_ascii_space;
    if (character == '>') return lcd_ascii_greater;
    if (character == ':') return lcd_ascii_colon;
    if (character == '-') return lcd_ascii_minus;
    if (character == '.') return lcd_ascii_period;
    if (character == '/') return lcd_ascii_slash;
    return lcd_ascii_question;
}

static void lcd_bus_init(void) {
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOD, &gpio);

    CLEAR_BIT(hspi1.Instance->CR1, SPI_CR1_SPE);
    MODIFY_REG(hspi1.Instance->CR2, SPI_CR2_TSIZE, 0);
    hspi1.Instance->IFCR =
        SPI_IFCR_EOTC |
        SPI_IFCR_TXTFC |
        SPI_IFCR_UDRC |
        SPI_IFCR_OVRC |
        SPI_IFCR_CRCEC |
        SPI_IFCR_TIFREC |
        SPI_IFCR_MODFC |
        SPI_IFCR_SUSPC;
    SET_BIT(hspi1.Instance->CR1, SPI_CR1_SPE);
    SET_BIT(hspi1.Instance->CR1, SPI_CR1_CSTART);
}

static void lcd_select(void) {
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void lcd_deselect(void) {
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void lcd_set_data_mode(uint8_t data_mode) {
    HAL_GPIO_WritePin(
        LCD_DC_GPIO_Port,
        LCD_DC_Pin,
        data_mode ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

static bsp_status_t lcd_spi_send(const uint8_t *data, uint16_t size) {
    uint32_t started_at = bsp_time_get_ms();

    while (size-- != 0U) {
        while ((hspi1.Instance->SR & SPI_SR_TXP) == 0U) {
            if (bsp_time_get_ms() - started_at >= LCD_SPI_TIMEOUT_MS) {
                return BSP_STATUS_TIMEOUT;
            }
        }
        *(__IO uint8_t *) &hspi1.Instance->TXDR = *data++;
    }

    while ((hspi1.Instance->SR & SPI_SR_TXC) == 0U) {
        if (bsp_time_get_ms() - started_at >= LCD_SPI_TIMEOUT_MS) {
            return BSP_STATUS_TIMEOUT;
        }
    }
    return BSP_STATUS_OK;
}

static bsp_status_t lcd_write_transaction(const uint8_t *data, uint16_t size) {
    lcd_select();
    bsp_status_t status = lcd_spi_send(data, size);
    lcd_deselect();
    return status;
}

static bsp_status_t lcd_write_command(uint8_t command, const uint8_t *data, uint16_t size) {
    lcd_set_data_mode(0);
    bsp_status_t status = lcd_write_transaction(&command, 1);
    if (status != BSP_STATUS_OK) return status;

    lcd_set_data_mode(1);
    for (uint16_t i = 0; i < size; ++i) {
        status = lcd_write_transaction(&data[i], 1);
        if (status != BSP_STATUS_OK) return status;
    }

    return status;
}

static bsp_status_t lcd_set_window(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    uint16_t x_start = (uint16_t) (x + LCD_X_OFFSET);
    uint16_t y_start = (uint16_t) (y + LCD_Y_OFFSET);
    uint16_t x_end = (uint16_t) (x_start + width - 1U);
    uint16_t y_end = (uint16_t) (y_start + height - 1U);
    uint8_t column_data[] = {
        (uint8_t) (x_start >> 8),
        (uint8_t) x_start,
        (uint8_t) (x_end >> 8),
        (uint8_t) x_end,
    };
    uint8_t row_data[] = {
        (uint8_t) (y_start >> 8),
        (uint8_t) y_start,
        (uint8_t) (y_end >> 8),
        (uint8_t) y_end,
    };

    bsp_status_t status = lcd_write_command(
        LCD_COMMAND_COLUMN_ADDRESS,
        column_data,
        sizeof(column_data)
    );
    if (status != BSP_STATUS_OK) return status;

    return lcd_write_command(LCD_COMMAND_ROW_ADDRESS, row_data, sizeof(row_data));
}

static bsp_status_t lcd_begin_pixel_write(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height
) {
    bsp_status_t status = lcd_set_window(x, y, width, height);
    if (status != BSP_STATUS_OK) return status;

    uint8_t command = LCD_COMMAND_MEMORY_WRITE;
    lcd_set_data_mode(0);
    status = lcd_write_transaction(&command, 1);
    if (status != BSP_STATUS_OK) return status;
    lcd_set_data_mode(1);

    return BSP_STATUS_OK;
}

static uint8_t lcd_area_is_valid(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    if (width == 0U || height == 0U) return 0;
    if (x >= BSP_LCD_WIDTH || y >= BSP_LCD_HEIGHT) return 0;
    if (width > BSP_LCD_WIDTH - x) return 0;
    if (height > BSP_LCD_HEIGHT - y) return 0;
    return 1;
}

static bsp_lcd_key_t lcd_decode_key(uint16_t raw) {
    if (raw < 6554U) return BSP_LCD_KEY_MIDDLE;
    if (raw < 19661U) return BSP_LCD_KEY_LEFT;
    if (raw < 32768U) return BSP_LCD_KEY_RIGHT;
    if (raw < 45875U) return BSP_LCD_KEY_UP;
    if (raw < 58982U) return BSP_LCD_KEY_DOWN;
    return BSP_LCD_KEY_NONE;
}

static uint8_t lcd_key_callback_is_set(
    bsp_lcd_key_t key,
    bsp_lcd_key_event_t event
) {
    unsigned long state = bsp_sys_enter_critical();
    uint8_t is_set = key_callbacks[key][event].callback != NULL;
    bsp_sys_exit_critical(state);
    return is_set;
}

static void lcd_dispatch_key_event(bsp_lcd_key_t key, bsp_lcd_key_event_t event) {
    unsigned long state = bsp_sys_enter_critical();
    lcd_key_callback_entry_t entry = key_callbacks[key][event];
    bsp_sys_exit_critical(state);

    if (entry.callback != NULL) entry.callback(key, event, entry.context);
}

static void lcd_handle_key_press(bsp_lcd_key_t key, uint32_t now) {
    if (key_tracker.pending_click_key != BSP_LCD_KEY_NONE) {
        uint32_t elapsed = now - key_tracker.pending_click_at;
        if (key_tracker.pending_click_key == key && elapsed < LCD_KEY_DOUBLE_CLICK_MS) {
            key_tracker.second_press = 1;
        } else {
            lcd_dispatch_key_event(key_tracker.pending_click_key, BSP_LCD_KEY_CLICK);
            key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
            key_tracker.second_press = 0;
        }
    } else {
        key_tracker.second_press = 0;
    }

    key_tracker.pressed_key = key;
    key_tracker.press_started_at = now;
    key_tracker.long_press_sent = 0;
}

static void lcd_handle_key_release(bsp_lcd_key_t key, uint32_t now) {
    if (key_tracker.pressed_key != key) return;

    if (key_tracker.long_press_sent) {
        key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
    } else if (
        key_tracker.second_press &&
        key_tracker.pending_click_key == key
    ) {
        key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
        lcd_dispatch_key_event(key, BSP_LCD_KEY_DOUBLE_CLICK);
    } else if (!lcd_key_callback_is_set(key, BSP_LCD_KEY_DOUBLE_CLICK)) {
        key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
        lcd_dispatch_key_event(key, BSP_LCD_KEY_CLICK);
    } else {
        key_tracker.pending_click_key = key;
        key_tracker.pending_click_at = now;
    }

    key_tracker.pressed_key = BSP_LCD_KEY_NONE;
    key_tracker.second_press = 0;
}

bsp_status_t bsp_lcd_init(void) {
    static const uint8_t memory_access_control[] = {0x00};
    static const uint8_t pixel_format[] = {0x05};
    static const uint8_t porch_control[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    static const uint8_t gate_control[] = {0x35};
    static const uint8_t vcom_setting[] = {0x32};
    static const uint8_t lcm_control[] = {0x01};
    static const uint8_t vrh_enable[] = {0x15};
    static const uint8_t vdv_setting[] = {0x20};
    static const uint8_t frame_rate_control[] = {0x0F};
    static const uint8_t power_control[] = {0xA4, 0xA1};
    static const uint8_t positive_voltage_gamma[] = {
        0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31,
        0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34,
    };
    static const uint8_t negative_voltage_gamma[] = {
        0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31,
        0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34,
    };

    lcd_initialized = 0;
    lcd_deselect();
    lcd_bus_init();
    bsp_lcd_set_backlight(0);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    bsp_time_delay(100);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
    bsp_lcd_set_backlight(1);
    bsp_time_delay(100);

    bsp_status_t status = lcd_write_command(LCD_COMMAND_SLEEP_OUT, NULL, 0);
    if (status != BSP_STATUS_OK) {
        bsp_lcd_set_backlight(0);
        return status;
    }
    bsp_time_delay(120);

    status = lcd_write_command(0x36, memory_access_control, sizeof(memory_access_control));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0x3A, pixel_format, sizeof(pixel_format));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xB2, porch_control, sizeof(porch_control));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xB7, gate_control, sizeof(gate_control));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xBB, vcom_setting, sizeof(vcom_setting));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xC2, lcm_control, sizeof(lcm_control));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xC3, vrh_enable, sizeof(vrh_enable));
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xC4, vdv_setting, sizeof(vdv_setting));
    if (status == BSP_STATUS_OK) {
        status = lcd_write_command(0xC6, frame_rate_control, sizeof(frame_rate_control));
    }
    if (status == BSP_STATUS_OK) status = lcd_write_command(0xD0, power_control, sizeof(power_control));
    if (status == BSP_STATUS_OK) {
        status = lcd_write_command(0xE0, positive_voltage_gamma, sizeof(positive_voltage_gamma));
    }
    if (status == BSP_STATUS_OK) {
        status = lcd_write_command(0xE1, negative_voltage_gamma, sizeof(negative_voltage_gamma));
    }
    if (status == BSP_STATUS_OK) {
        status = lcd_write_command(LCD_COMMAND_INVERSION_ON, NULL, 0);
    }
    if (status == BSP_STATUS_OK) {
        status = lcd_write_command(LCD_COMMAND_DISPLAY_ON, NULL, 0);
    }
    if (status != BSP_STATUS_OK) {
        bsp_lcd_set_backlight(0);
        return status;
    }

    lcd_initialized = 1;
    status = bsp_lcd_clear(BSP_LCD_COLOR_BLACK);
    if (status != BSP_STATUS_OK) {
        lcd_initialized = 0;
        bsp_lcd_set_backlight(0);
        return status;
    }
    bsp_lcd_set_backlight(1);
    return BSP_STATUS_OK;
}

void bsp_lcd_set_backlight(uint8_t enabled) {
    HAL_GPIO_WritePin(
        LCD_BLK_GPIO_Port,
        LCD_BLK_Pin,
        enabled ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

bsp_status_t bsp_lcd_clear(bsp_lcd_color_t color) {
    return bsp_lcd_fill(0, 0, BSP_LCD_WIDTH, BSP_LCD_HEIGHT, color);
}

bsp_status_t bsp_lcd_fill(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    bsp_lcd_color_t color
) {
    if (!lcd_initialized || !lcd_area_is_valid(x, y, width, height)) {
        return BSP_STATUS_ERROR;
    }

    uint8_t buffer[LCD_PIXEL_BUFFER_SIZE];
    for (uint16_t i = 0; i < LCD_PIXEL_BUFFER_SIZE; i += 2U) {
        buffer[i] = (uint8_t) (color >> 8);
        buffer[i + 1U] = (uint8_t) color;
    }

    bsp_status_t status = lcd_begin_pixel_write(x, y, width, height);
    if (status != BSP_STATUS_OK) return status;

    uint32_t bytes_remaining = (uint32_t) width * height * 2U;
    while (bytes_remaining != 0U) {
        uint16_t chunk_size = bytes_remaining > LCD_PIXEL_BUFFER_SIZE
            ? LCD_PIXEL_BUFFER_SIZE
            : (uint16_t) bytes_remaining;
        status = lcd_write_transaction(buffer, chunk_size);
        if (status != BSP_STATUS_OK) break;
        bytes_remaining -= chunk_size;
    }

    return status;
}

bsp_status_t bsp_lcd_draw_pixel(uint16_t x, uint16_t y, bsp_lcd_color_t color) {
    return bsp_lcd_fill(x, y, 1, 1, color);
}

bsp_status_t bsp_lcd_draw_line(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    bsp_lcd_color_t color
) {
    if (
        !lcd_initialized ||
        x0 >= BSP_LCD_WIDTH ||
        x1 >= BSP_LCD_WIDTH ||
        y0 >= BSP_LCD_HEIGHT ||
        y1 >= BSP_LCD_HEIGHT
    ) {
        return BSP_STATUS_ERROR;
    }
    if (y0 == y1) {
        uint16_t start = x0 < x1 ? x0 : x1;
        uint16_t width = (uint16_t) (x0 < x1 ? x1 - x0 + 1U : x0 - x1 + 1U);
        return bsp_lcd_fill(start, y0, width, 1, color);
    }
    if (x0 == x1) {
        uint16_t start = y0 < y1 ? y0 : y1;
        uint16_t height = (uint16_t) (y0 < y1 ? y1 - y0 + 1U : y0 - y1 + 1U);
        return bsp_lcd_fill(x0, start, 1, height, color);
    }

    int32_t current_x = x0;
    int32_t current_y = y0;
    int32_t end_x = x1;
    int32_t end_y = y1;
    int32_t delta_x = end_x > current_x ? end_x - current_x : current_x - end_x;
    int32_t delta_y = end_y > current_y ? end_y - current_y : current_y - end_y;
    int32_t step_x = current_x < end_x ? 1 : -1;
    int32_t step_y = current_y < end_y ? 1 : -1;
    int32_t error = delta_x - delta_y;

    while (1) {
        bsp_status_t status = bsp_lcd_draw_pixel(
            (uint16_t) current_x,
            (uint16_t) current_y,
            color
        );
        if (status != BSP_STATUS_OK) return status;
        if (current_x == end_x && current_y == end_y) return BSP_STATUS_OK;

        int32_t doubled_error = error * 2;
        if (doubled_error > -delta_y) {
            error -= delta_y;
            current_x += step_x;
        }
        if (doubled_error < delta_x) {
            error += delta_x;
            current_y += step_y;
        }
    }
}

bsp_status_t bsp_lcd_draw_rect(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    bsp_lcd_color_t color
) {
    if (!lcd_initialized || !lcd_area_is_valid(x, y, width, height)) {
        return BSP_STATUS_ERROR;
    }
    if (width == 1U || height == 1U) return bsp_lcd_fill(x, y, width, height, color);

    bsp_status_t status = bsp_lcd_fill(x, y, width, 1, color);
    if (status == BSP_STATUS_OK) {
        status = bsp_lcd_fill(x, (uint16_t) (y + height - 1U), width, 1, color);
    }
    if (status == BSP_STATUS_OK && height > 2U) {
        status = bsp_lcd_fill(x, (uint16_t) (y + 1U), 1, (uint16_t) (height - 2U), color);
    }
    if (status == BSP_STATUS_OK && height > 2U) {
        status = bsp_lcd_fill(
            (uint16_t) (x + width - 1U),
            (uint16_t) (y + 1U),
            1,
            (uint16_t) (height - 2U),
            color
        );
    }
    return status;
}

bsp_status_t bsp_lcd_draw_circle(
    uint16_t center_x,
    uint16_t center_y,
    uint16_t radius,
    bsp_lcd_color_t color
) {
    if (
        !lcd_initialized ||
        center_x < radius ||
        center_y < radius ||
        (uint32_t) center_x + radius >= BSP_LCD_WIDTH ||
        (uint32_t) center_y + radius >= BSP_LCD_HEIGHT
    ) {
        return BSP_STATUS_ERROR;
    }

    int32_t x = radius;
    int32_t y = 0;
    int32_t error = 1 - (int32_t) radius;

    while (x >= y) {
        const int32_t points[][2] = {
            {(int32_t) center_x + x, (int32_t) center_y + y},
            {(int32_t) center_x + y, (int32_t) center_y + x},
            {(int32_t) center_x - y, (int32_t) center_y + x},
            {(int32_t) center_x - x, (int32_t) center_y + y},
            {(int32_t) center_x - x, (int32_t) center_y - y},
            {(int32_t) center_x - y, (int32_t) center_y - x},
            {(int32_t) center_x + y, (int32_t) center_y - x},
            {(int32_t) center_x + x, (int32_t) center_y - y},
        };

        for (uint16_t i = 0; i < 8U; ++i) {
            bsp_status_t status = bsp_lcd_draw_pixel(
                (uint16_t) points[i][0],
                (uint16_t) points[i][1],
                color
            );
            if (status != BSP_STATUS_OK) return status;
        }

        ++y;
        if (error < 0) {
            error += 2 * y + 1;
        } else {
            --x;
            error += 2 * (y - x) + 1;
        }
    }

    return BSP_STATUS_OK;
}

static bsp_lcd_color_t lcd_ascii_pixel_color(
    const uint8_t *glyph,
    uint16_t glyph_x,
    uint16_t glyph_y,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    bsp_lcd_color_t shadow,
    bsp_lcd_ascii_font_t font
) {
    uint8_t glyph_pixel =
        glyph_x < LCD_ASCII_GLYPH_WIDTH &&
        glyph_y < LCD_ASCII_GLYPH_HEIGHT &&
        (glyph[glyph_x] & (1U << glyph_y)) != 0U;
    if (font == BSP_LCD_ASCII_FONT_CLASSIC) {
        return glyph_pixel ? foreground : background;
    }

    uint8_t previous_column_pixel =
        glyph_x > 0U &&
        glyph_x <= LCD_ASCII_GLYPH_WIDTH &&
        glyph_y < LCD_ASCII_GLYPH_HEIGHT &&
        (glyph[glyph_x - 1U] & (1U << glyph_y)) != 0U;
    if (glyph_pixel || previous_column_pixel) return foreground;

    uint8_t upper_left_pixel =
        glyph_x > 0U &&
        glyph_x <= LCD_ASCII_GLYPH_WIDTH &&
        glyph_y > 0U &&
        glyph_y <= LCD_ASCII_GLYPH_HEIGHT &&
        (glyph[glyph_x - 1U] & (1U << (glyph_y - 1U))) != 0U;
    return upper_left_pixel ? shadow : background;
}

static bsp_status_t lcd_draw_ascii_text(
    uint16_t x,
    uint16_t y,
    const char *text,
    size_t length,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    bsp_lcd_ascii_font_t font,
    uint8_t scale
) {
    if (
        text == NULL ||
        font < BSP_LCD_ASCII_FONT_CLASSIC ||
        font >= BSP_LCD_ASCII_FONT_COUNT ||
        scale == 0U ||
        scale > LCD_ASCII_MAX_SCALE ||
        !lcd_initialized ||
        x >= BSP_LCD_WIDTH ||
        y >= BSP_LCD_HEIGHT
    ) {
        return BSP_STATUS_ERROR;
    }

    const uint16_t character_width = (uint16_t) (LCD_ASCII_CELL_WIDTH * scale);
    const uint16_t height = (uint16_t) (LCD_ASCII_CELL_HEIGHT * scale);
    if (
        height > BSP_LCD_HEIGHT - y ||
        length > (BSP_LCD_WIDTH - x) / character_width
    ) {
        return BSP_STATUS_ERROR;
    }
    if (length == 0U) return BSP_STATUS_OK;

    const uint16_t width = (uint16_t) (length * character_width);
    bsp_status_t status = lcd_begin_pixel_write(x, y, width, height);
    if (status != BSP_STATUS_OK) return status;

    uint8_t buffer[LCD_PIXEL_BUFFER_SIZE];
    uint16_t buffer_size = 0;
    const bsp_lcd_color_t shadow = (bsp_lcd_color_t) (
        (foreground >> 1U) & 0x7BEFU
    );

    for (uint16_t pixel_y = 0; pixel_y < height; ++pixel_y) {
        uint16_t glyph_y = pixel_y / scale;
        for (size_t index = 0; index < length; ++index) {
            const uint8_t *glyph = lcd_ascii_glyph(text[index]);
            for (
                uint16_t pixel_x = 0;
                pixel_x < character_width;
                ++pixel_x
            ) {
                bsp_lcd_color_t color = lcd_ascii_pixel_color(
                    glyph,
                    pixel_x / scale,
                    glyph_y,
                    foreground,
                    background,
                    shadow,
                    font
                );
                buffer[buffer_size++] = (uint8_t) (color >> 8);
                buffer[buffer_size++] = (uint8_t) color;
                if (buffer_size == LCD_PIXEL_BUFFER_SIZE) {
                    status = lcd_write_transaction(buffer, buffer_size);
                    if (status != BSP_STATUS_OK) return status;
                    buffer_size = 0;
                }
            }
        }
    }

    if (buffer_size != 0U) {
        status = lcd_write_transaction(buffer, buffer_size);
    }
    return status;
}

bsp_status_t bsp_lcd_draw_ascii_char_font(
    uint16_t x,
    uint16_t y,
    char character,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    bsp_lcd_ascii_font_t font,
    uint8_t scale
) {
    return lcd_draw_ascii_text(
        x,
        y,
        &character,
        1,
        foreground,
        background,
        font,
        scale
    );
}

bsp_status_t bsp_lcd_draw_ascii_char(
    uint16_t x,
    uint16_t y,
    char character,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    uint8_t scale
) {
    return bsp_lcd_draw_ascii_char_font(
        x,
        y,
        character,
        foreground,
        background,
        BSP_LCD_ASCII_FONT_CLASSIC,
        scale
    );
}

bsp_status_t bsp_lcd_draw_ascii_font(
    uint16_t x,
    uint16_t y,
    const char *text,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    bsp_lcd_ascii_font_t font,
    uint8_t scale
) {
    if (text == NULL) return BSP_STATUS_ERROR;
    return lcd_draw_ascii_text(
        x,
        y,
        text,
        strlen(text),
        foreground,
        background,
        font,
        scale
    );
}

bsp_status_t bsp_lcd_draw_ascii(
    uint16_t x,
    uint16_t y,
    const char *text,
    bsp_lcd_color_t foreground,
    bsp_lcd_color_t background,
    uint8_t scale
) {
    return bsp_lcd_draw_ascii_font(
        x,
        y,
        text,
        foreground,
        background,
        BSP_LCD_ASCII_FONT_CLASSIC,
        scale
    );
}

bsp_status_t bsp_lcd_blit_rgb565(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    const bsp_lcd_color_t *pixels
) {
    if (
        !lcd_initialized ||
        pixels == NULL ||
        !lcd_area_is_valid(x, y, width, height)
    ) {
        return BSP_STATUS_ERROR;
    }

    bsp_status_t status = lcd_begin_pixel_write(x, y, width, height);
    if (status != BSP_STATUS_OK) return status;

    uint8_t buffer[LCD_PIXEL_BUFFER_SIZE];
    uint32_t pixels_remaining = (uint32_t) width * height;
    while (pixels_remaining != 0U) {
        uint16_t pixel_count = pixels_remaining > LCD_PIXEL_BUFFER_SIZE / 2U
            ? LCD_PIXEL_BUFFER_SIZE / 2U
            : (uint16_t) pixels_remaining;
        for (uint16_t i = 0; i < pixel_count; ++i) {
            buffer[i * 2U] = (uint8_t) (pixels[i] >> 8);
            buffer[i * 2U + 1U] = (uint8_t) pixels[i];
        }

        status = lcd_write_transaction(buffer, (uint16_t) (pixel_count * 2U));
        if (status != BSP_STATUS_OK) break;
        pixels += pixel_count;
        pixels_remaining -= pixel_count;
    }

    return status;
}

bsp_status_t bsp_lcd_set_key_callback(
    bsp_lcd_key_t key,
    bsp_lcd_key_event_t event,
    bsp_lcd_key_callback_t callback,
    void *context
) {
    if (key >= BSP_LCD_KEY_COUNT || event >= BSP_LCD_KEY_EVENT_COUNT) {
        return BSP_STATUS_ERROR;
    }

    unsigned long state = bsp_sys_enter_critical();
    key_callbacks[key][event].callback = callback;
    key_callbacks[key][event].context = context;
    bsp_sys_exit_critical(state);
    return BSP_STATUS_OK;
}

void bsp_lcd_process_keys(void) {
    uint32_t now = bsp_time_get_ms();
    bsp_lcd_key_t sampled_key = lcd_decode_key(bsp_adc_lcd_key_raw());

    if (!key_tracker.initialized) {
        key_tracker.candidate_key = sampled_key;
        key_tracker.stable_key = BSP_LCD_KEY_NONE;
        key_tracker.pressed_key = BSP_LCD_KEY_NONE;
        key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
        key_tracker.candidate_since = now;
        key_tracker.initialized = 1;
        return;
    }

    if (sampled_key != key_tracker.candidate_key) {
        key_tracker.candidate_key = sampled_key;
        key_tracker.candidate_since = now;
    }

    if (
        key_tracker.candidate_key != key_tracker.stable_key &&
        now - key_tracker.candidate_since >= LCD_KEY_DEBOUNCE_MS
    ) {
        bsp_lcd_key_t previous_key = key_tracker.stable_key;
        key_tracker.stable_key = key_tracker.candidate_key;

        if (previous_key != BSP_LCD_KEY_NONE) {
            lcd_handle_key_release(previous_key, now);
        }
        if (key_tracker.stable_key != BSP_LCD_KEY_NONE) {
            lcd_handle_key_press(key_tracker.stable_key, now);
        }
    }

    if (
        key_tracker.stable_key != BSP_LCD_KEY_NONE &&
        !key_tracker.long_press_sent &&
        now - key_tracker.press_started_at >= LCD_KEY_LONG_PRESS_MS
    ) {
        if (
            key_tracker.second_press &&
            key_tracker.pending_click_key == key_tracker.stable_key
        ) {
            key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
            key_tracker.second_press = 0;
        }
        key_tracker.long_press_sent = 1;
        lcd_dispatch_key_event(key_tracker.stable_key, BSP_LCD_KEY_LONG_PRESS);
    }

    if (
        key_tracker.pending_click_key != BSP_LCD_KEY_NONE &&
        !key_tracker.second_press &&
        now - key_tracker.pending_click_at >= LCD_KEY_DOUBLE_CLICK_MS
    ) {
        bsp_lcd_key_t key = key_tracker.pending_click_key;
        key_tracker.pending_click_key = BSP_LCD_KEY_NONE;
        lcd_dispatch_key_event(key, BSP_LCD_KEY_CLICK);
    }
}
