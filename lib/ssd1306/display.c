#include "display.h"

void init_display(ssd1306_t *ssd)
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(SSD1306_I2C_PORT, 400 * 1000);

    gpio_set_function(SSD1306_I2C_SDA, GPIO_FUNC_I2C);                          // Set the GPIO pin function to I2C
    gpio_set_function(SSD1306_I2C_SCL, GPIO_FUNC_I2C);                          // Set the GPIO pin function to I2C
    gpio_pull_up(SSD1306_I2C_SDA);                                              // Pull up the data line
    gpio_pull_up(SSD1306_I2C_SCL);                                              // Pull up the clock line
                                                                                // Inicializa a estrutura do display
    ssd1306_init(ssd, WIDTH, HEIGHT, false, SSD1306_ADDRESS, SSD1306_I2C_PORT); // Inicializa o display
    ssd1306_config(ssd);                                                        // Configura o display
    ssd1306_send_data(ssd);

    ssd1306_init(ssd, WIDTH, HEIGHT, false, SSD1306_ADDRESS, SSD1306_I2C_PORT); // Inicializa o display
    ssd1306_config(ssd);                                                        // Configura o display
    ssd1306_send_data(ssd);                                                     // Envia os dados para o display
    ssd1306_fill(ssd, false);                                                   // Limpa o display
    ssd1306_send_data(ssd);
}

void draw_centered_text(ssd1306_t *ssd, const char *text, int y)
{
    int x = (128 - (strlen(text) * 8)) / 2; // Calcula a posição X para centralizar
    ssd1306_draw_string(ssd, text, x, y);   // Desenha o texto na posição calculada
}