#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lib/ssd1306/ssd1306.h"
#include "lib/ssd1306/display.h"
#include "lib/led/led.h"
#include "lib/button/button.h"
#include "lib/ws2812b/ws2812b.h"
#include "lib/buzzer/buzzer.h"

#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define tam_quad 10

// Estruturas de dados
typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t; // Estrutura para armazenar os dados do joystick

typedef enum
{
    STATUS_NORMAL,
    STATUS_ALERT
} flood_status_t; // Enumeração para o status de cheia/inundação

// Protótipos das funções
void vJoystickTask(void *pvParameters);            // Task para leitura do joystick
void vDisplayTask(void *pvParameters);             // Task para exibição no display
void vLedGreenTask(void *pvParameters);            // Task para controle do LED verde
void vLedBlueTask(void *pvParameters);             // Task para controle do LED azul
void gpio_irq_handler(uint gpio, uint32_t events); // Função de interrupção para o botão B
void calculate_flood_percentages(
    joystick_data_t *joydata,
    float *rain_volume_percent,
    float *water_level_percent); // Função para calcular a porcentagem de água e chuva
flood_status_t get_flood_alert_status(
    uint16_t rain_level_raw,
    uint16_t water_level_raw); // Função para verificar o status de cheia/inundação

// Variáveis globais
QueueHandle_t xQueueJoystickData; // Fila para compartilhar dados do joystick

int main()
{
    // Ativa BOOTSEL via botão B
    init_btn(BTN_B_PIN);
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedGreenTask, "LED red Task", 256, NULL, 1, NULL);
    xTaskCreate(vLedBlueTask, "LED blue Task", 256, NULL, 1, NULL);

    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}

// Função de interrupção para o botão B
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Task para leitura do joystick
void vJoystickTask(void *params)
{
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

// Task para exibição no display
void vDisplayTask(void *params)
{
    ssd1306_t ssd;
    joystick_data_t joydata;
    bool color = true;
    float rain_volume_percent = 0.0f;
    float water_level_percent = 0.0f;
    char water_level_str[10];
    char rain_volume_str[10];
    flood_status_t flood_status = STATUS_NORMAL;

    init_display(&ssd); // Inicializa o display

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Verifica o status de cheia/inundação
            calculate_flood_percentages(&joydata, &rain_volume_percent, &water_level_percent);
            flood_status = get_flood_alert_status(rain_volume_percent, water_level_percent);

            snprintf(water_level_str, sizeof(water_level_str), "%.2f%%", water_level_percent);
            snprintf(rain_volume_str, sizeof(rain_volume_str), "%.2f%%", rain_volume_percent);

            ssd1306_fill(&ssd, !color);                        // Limpa a tela
            ssd1306_rect(&ssd, 0, 0, 128, 64, color, false);    // Desenha o retângulo
            ssd1306_draw_string(&ssd, "N. Agua:", 5, 18);      // Exibe o título
            ssd1306_draw_string(&ssd, water_level_str, 5, 28); // Exibe o nível de água
            ssd1306_draw_string(&ssd, "V. chuva:", 5, 42);     // Exibe o título
            ssd1306_draw_string(&ssd, rain_volume_str, 5, 52); // Exibe o volume de chuva

            if (flood_status == STATUS_ALERT)
                draw_centered_text(&ssd, "!! ALERTA !!", 4); // Exibe alerta
            else
                draw_centered_text(&ssd, "NORMAL", 4); // Exibe normal

            ssd1306_send_data(&ssd);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 100ms
    }
}

// Task para controle do LED verde
void vLedGreenTask(void *params)
{
    gpio_set_function(GREEN_LED_PIN, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(GREEN_LED_PIN); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                          // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);          // Duty inicial
    pwm_set_enabled(slice, true);                      // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t center_desviation = (int16_t)joydata.x_pos - 2000;
            if (center_desviation < 0)
                center_desviation = -center_desviation;
            uint16_t pwm_value = (center_desviation * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_B, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

// Task para controle do LED azul
void vLedBlueTask(void *params)
{
    gpio_set_function(BLUE_LED_PIN, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(BLUE_LED_PIN); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                         // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);         // Duty inicial
    pwm_set_enabled(slice, true);                     // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t center_deviation = (int16_t)joydata.y_pos - 2048;
            if (center_deviation < 0)
                center_deviation = -center_deviation;
            uint16_t pwm_value = (center_deviation * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_A, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

// Função para calcular a porcentagem de água e chuva
void calculate_flood_percentages(joystick_data_t *joydata, float *rain_volume_percent, float *water_level_percent)
{
    // Considerando joystick centralizado em 2048, normaliza para 0~4095
    uint16_t rain_level = (joydata->x_pos > 2048) ? (joydata->x_pos - 2048) * 2 : (2048 - joydata->x_pos) * 2;
    uint16_t water_level = (joydata->y_pos > 2048) ? (joydata->y_pos - 2048) * 2 : (2048 - joydata->y_pos) * 2;

    // Calcula o nível da água e o volume de chuva em porcentagem
    *rain_volume_percent = (rain_level / 4095.0f) * 100.0f;
    *water_level_percent = (water_level / 4095.0f) * 100.0f;
}

// Função para tratar os dados do joystick e verificar o status de cheia/inundação
flood_status_t get_flood_alert_status(uint16_t rain_volume_percent, uint16_t water_level_percent)
{
    if (water_level_percent >= 70.0f || rain_volume_percent >= 80.0f)
        return STATUS_ALERT;
    else
        return STATUS_NORMAL;
}
