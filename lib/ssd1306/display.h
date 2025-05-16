#ifndef SSD1306_DISPLAY_H
#define SSD1306_DISPLAY_H

#include <stdlib.h>
#include "pico/stdlib.h"
#include "ssd1306.h"
#include "string.h"

#define SSD1306_I2C_PORT i2c1
#define SSD1306_I2C_SDA 14
#define SSD1306_I2C_SCL 15
#define SSD1306_ADDRESS 0x3C

void init_display(ssd1306_t *ssd);
void draw_centered_text(ssd1306_t *ssd, const char *text, int y);

#endif // SSD1306_DISPLAY_H
