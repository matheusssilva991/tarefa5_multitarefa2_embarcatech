#include "led.h"

void init_led(uint8_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
}

void init_leds()
{
    init_led(GREEN_LED_PIN);
    init_led(BLUE_LED_PIN);
    init_led(RED_LED_PIN);
}

void turn_off_leds()
{
    gpio_put(GREEN_LED_PIN, false);
    gpio_put(BLUE_LED_PIN, false);
    gpio_put(RED_LED_PIN, false);
}

void set_led_green()
{
    turn_off_leds();
    gpio_put(GREEN_LED_PIN, true);
}

void set_led_blue()
{
    turn_off_leds();
    gpio_put(BLUE_LED_PIN, true);
}

void set_led_red()
{
    turn_off_leds();
    gpio_put(RED_LED_PIN, true);
}

void set_led_yellow()
{
    turn_off_leds();
    gpio_put(GREEN_LED_PIN, true);
    gpio_put(RED_LED_PIN, true);
}