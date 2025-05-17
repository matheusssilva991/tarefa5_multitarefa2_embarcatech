# **Estação de Alerta de Enchente com Simulação por Joystick**

## **Descrição**

Este projeto é uma estação embarcada para monitoramento de cheias e inundações, utilizando sensores analógicos (simulados por joystick), display OLED, matriz de LEDs, buzzer e LED RGB. O sistema monitora em tempo real o nível da água e o volume de chuva, sinalizando automaticamente situações de alerta por meio de interface visual e sonora.

---

## **Funcionalidades**

- **Monitoramento Ambiental:**
  - Leitura contínua do nível da água e do volume de chuva (via joystick analógico).
  - Cálculo e exibição dos valores em porcentagem no display OLED.
- **Alerta Automático:**
  - Modo alerta ativado automaticamente quando o nível da água ≥ 70% ou o volume de chuva ≥ 80%.
  - LED RGB indica o status (verde para normal, vermelho para alerta).
  - Buzzer emite sinais sonoros distintos para cada tipo de alerta.
  - Matriz de LEDs pode exibir ícone ou símbolo de perigo em modo alerta.
  - Display OLED destaca o status de alerta.
- **Interface Física:**
  - Botão para funções de controle/reset.
  - Display OLED SSD1306 (128x64 pixels).
  - Matriz de LEDs WS2812B.
  - Buzzer para sinalização sonora.
  - LED RGB para indicação visual.

---

## **Requisitos**

### **Hardware**

- Microcontrolador Raspberry Pi Pico W.
- Display OLED SSD1306 (128x64 pixels).
- Matriz de LEDs WS2812B.
- Buzzer.
- LED RGB (ou LEDs individuais).
- Joystick analógico (ou dois potenciômetros para simular sensores).
- Botão para controle/reset.
- Fonte de alimentação compatível.

### **Software**

- **SDK do Raspberry Pi Pico.**
- **FreeRTOS** para gerenciamento de tarefas.
- **Bibliotecas adicionais:**
  - `ssd1306` para controle do display OLED.
  - `ws2812b` para controle da matriz de LEDs.
  - `button` para leitura de botões.
  - `buzzer` para controle do buzzer.
  - `led` para controle do LED RGB.

---

## **Como Rodar**

1. **Clone o repositório:**

   ```bash
   git clone https://github.com/matheusssilva991/tarefa5_multitarefa2_embarcatech.git
   cd estacao-monitoramento-cheias
   ```

2. **Configure o caminho do FreeRTOS:**
   - Edite o arquivo `CMakeLists.txt` e defina o caminho correto para o FreeRTOS:

   ```cmake
   set(FREERTOS_PATH "/caminho/para/seu/FreeRTOS")
   ```

   - Certifique-se de que o FreeRTOS esteja corretamente instalado e acessível.
   - O caminho padrão é `~/pico/FreeRTOS`.
   - Caso tenha instalado o FreeRTOS em outro local, ajuste o caminho conforme necessário.

3. **Compile e envie o código para o Raspberry Pi Pico W:**

   ```bash
   mkdir build
   cd build
   cmake -G "Ninja" ..
   ninja
   ```

4. **Conecte o hardware e alimente o Raspberry Pi Pico W.**
   - O sistema iniciará automaticamente e exibirá os dados no display OLED.

---

## **Demonstração**

Confira o vídeo de demonstração do projeto no YouTube:
[![Demonstração da Estação de Monitoramento de Cheias]](https://drive.google.com/file/d/18Y9qw2nJHHx6MuAKeCkl1dMBZXa8QG9K/view?usp=sharing)

---

## **Contribuindo**

1. Faça um fork deste repositório.
2. Crie uma nova branch: `git checkout -b minha-contribuicao`.
3. Faça suas alterações e commit: `git commit -m 'Minha contribuição'`.
4. Envie para o seu fork: `git push origin minha-contribuicao`.
5. Abra um Pull Request neste repositório.

---

## **Licença**

Este projeto está licenciado sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para mais detalhes.
