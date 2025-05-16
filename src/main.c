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
#define LED_BLUE 12
#define LED_GREEN 11
#define tam_quad 10

// Estrutura para armazenar os dados do joystick
// (posição X e Y)
typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

// Protótipos das funções
void vJoystickTask(void *pvParameters);            // Task para leitura do joystick
void vDisplayTask(void *pvParameters);             // Task para exibição no display
void vLedGreenTask(void *pvParameters);            // Task para controle do LED verde
void vLedBlueTask(void *pvParameters);             // Task para controle do LED azul
void gpio_irq_handler(uint gpio, uint32_t events); // Função de interrupção para o botão B

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
    init_display(&ssd); // Inicializa o display

    joystick_data_t joydata;
    bool cor = true;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            uint8_t x = (joydata.x_pos * (128 - tam_quad)) / 4095;
            uint8_t y = (joydata.y_pos * (64 - tam_quad)) / 4095;
            y = (64 - tam_quad) - y;                                 // Inverte o eixo Y
            ssd1306_fill(&ssd, !cor);                                // Limpa a tela
            ssd1306_rect(&ssd, y, x, tam_quad, tam_quad, cor, !cor); // Quadrado 5x5
            ssd1306_send_data(&ssd);
        }
    }
}

// Task para controle do LED verde
void vLedGreenTask(void *params)
{
    gpio_set_function(LED_GREEN, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(LED_GREEN); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                      // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);      // Duty inicial
    pwm_set_enabled(slice, true);                  // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t desvio_centro = (int16_t)joydata.x_pos - 2000;
            if (desvio_centro < 0)
                desvio_centro = -desvio_centro;
            uint16_t pwm_value = (desvio_centro * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_B, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

// Task para controle do LED azul
void vLedBlueTask(void *params)
{
    gpio_set_function(LED_BLUE, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(LED_BLUE); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                     // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);     // Duty inicial
    pwm_set_enabled(slice, true);                 // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t desvio_centro = (int16_t)joydata.y_pos - 2048;
            if (desvio_centro < 0)
                desvio_centro = -desvio_centro;
            uint16_t pwm_value = (desvio_centro * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_A, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}
