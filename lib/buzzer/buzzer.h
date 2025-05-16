#ifndef BUZZER_H
#define BUZZER_H

#include <stdlib.h>
#include "pico/stdlib.h"

#define BUZZER_A_PIN 21 // GPIO para buzzer A
#define BUZZER_B_PIN 10 // GPIO para buzzer B

int init_buzzer(uint pin, float clk_div); // Inicializa o PWM no pino do buzzer
void play_tone(uint pin, uint frequency); // Toca uma nota com a frequência e duração especificadas
void stop_tone(uint pin);                 // Desliga o tom no pino do buzzer

#endif // BUZZER_H
