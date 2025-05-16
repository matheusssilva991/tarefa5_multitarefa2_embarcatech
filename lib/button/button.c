#include "button.h"

void init_btn(uint8_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void init_btns()
{
    init_btn(BTN_A_PIN);
    init_btn(BTN_B_PIN);
}

bool btn_is_pressed(uint8_t pin)
{
    return !gpio_get(pin); // Retorna verdadeiro se o botão estiver pressionado
}
