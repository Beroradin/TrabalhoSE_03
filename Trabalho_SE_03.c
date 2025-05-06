// Trabalho SE 03 - Semáforo Inteligente utilizando FreeRTOS
// Bibliotecas utilizadas
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2818b.pio.h"
#include "ssd1306.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"  // É possível alterar quantidade de prioridades e outras coisas aqui
#include "task.h"

// Definições e constantes
#define I2C_PORT i2c1  // Utilizado o canal 2 do I2C
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define RED_PIN    13
#define BLUE_PIN   12
#define GREEN_PIN  11
#define BOTAO_A    5
#define BUZZER 21
#define LED_COUNT 25
#define LED_PIN 7

// Definindo a frequência desejada
#define PWM_FREQ_LED   20000 // 20 kHz
#define PWM_FREQ_BUZZER 1000  // 1 kHz
#define PWM_WRAP   255   // 8 bits de wrap (256 valores)

// Estados do semáforo
#define ESTADO_VERDE   0
#define ESTADO_AMARELO 1
#define ESTADO_VERMELHO 2
#define ESTADO_DESLIGADO 3

// Flags e Variáveis globais
volatile bool modoNoturno = false;    // false = modo normal, true = modo noturno
volatile bool estadoBotao = true;     // Estado anterior do botão
volatile TickType_t ultimaPressionadoBotao = 0; // Para implementar debounce
volatile int state = ESTADO_DESLIGADO; // Estado atual do semáforo na matriz de LED
volatile int displayState = ESTADO_DESLIGADO; // Estado que o display deve mostrar
ssd1306_t ssd;           // Inicializa a estrutura do display
volatile bool cor = true;
char str_y[5];

// Definição da estrutura do pixel
struct pixel_t {
    uint8_t G, R, B;
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;

// Confiugarações das máquinas de estados do PIO
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;

// Protótipos de funções
void initSettings(void);
void initssd1306();
void configurarLEDs(uint32_t vermelho, uint32_t azul, uint32_t verde);
void vLedRGBNormal(void *pvParameters);
void vBotaoTask(void *pvParameters);
void configurarBuzzer(uint32_t volume);
void vBuzzerNormal(void *pvParameters);
void vLedMatrixNormal(void *pvParameters);
void vDisplayNormal(void *pvParameters);
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b);
void npWrite();
void npDisplayDigit(int digit);
void npClear();
void npInit(uint pin);
int getIndex(int x, int y);

// Matrizes para cada dígito 
const uint8_t states[4][5][5][3] = {
    // Verde rbg 192, 8, 100 100, 192, 8
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}, 
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},    
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},    
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},    
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}  
    },
    // Amarelo
    {
        {{0, 0, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}},
        {{120, 40, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {120, 40, 0}},
        {{0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Vermelho
    {
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}} 
    },
    // Desligado
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},   
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}} 
    },
};

int main() {
    // Inicializa stdio
    stdio_init_all();
    printf("Iniciando aplicação de semáforo\n");
    npInit(LED_PIN);
    
    // Cria as tarefas
    xTaskCreate(vLedRGBNormal, "Modo Normal", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);     
    xTaskCreate(vBotaoTask, "Tarefa Botão", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vBuzzerNormal, "Buzzer Normal", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLedMatrixNormal, "Led Matrix Normal", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vDisplayNormal, "Display Normal", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Inicia o escalonador do FreeRTOS
    vTaskStartScheduler();
    panic_unsupported();
}

// Função para configurar os LEDs e os valores PWM dos mesmos
void configurarLEDs(uint32_t vermelho, uint32_t azul, uint32_t verde) {
    pwm_set_gpio_level(RED_PIN, vermelho);
    pwm_set_gpio_level(BLUE_PIN, azul);
    pwm_set_gpio_level(GREEN_PIN, verde);
}

// Função para configurar o volume do buzzer - frequência de 1 KHz
void configurarBuzzer(uint32_t volume) {
    pwm_set_gpio_level(BUZZER, volume);
}

void initSettings(){
    // Inicializa o pino do botão
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    
    // Inicializa os pinos PWM para os LEDs
    gpio_set_function(RED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BLUE_PIN, GPIO_FUNC_PWM);
    gpio_set_function(GREEN_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER, GPIO_FUNC_PWM);
    
    // Obtém os números dos canais PWM para os pinos
    uint slice_num_red = pwm_gpio_to_slice_num(RED_PIN);
    uint slice_num_blue = pwm_gpio_to_slice_num(BLUE_PIN);
    uint slice_num_green = pwm_gpio_to_slice_num(GREEN_PIN);
    uint slice_num_buzzer = pwm_gpio_to_slice_num(BUZZER);
    
    // Configuração da frequência PWM
    pwm_set_clkdiv(slice_num_red, (float)clock_get_hz(clk_sys) / PWM_FREQ_LED / (PWM_WRAP + 1));
    pwm_set_clkdiv(slice_num_blue, (float)clock_get_hz(clk_sys) / PWM_FREQ_LED / (PWM_WRAP + 1));
    pwm_set_clkdiv(slice_num_green, (float)clock_get_hz(clk_sys) / PWM_FREQ_LED / (PWM_WRAP + 1));
    pwm_set_clkdiv(slice_num_buzzer, (float)clock_get_hz(clk_sys) / PWM_FREQ_BUZZER / (PWM_WRAP + 1));
    
    // Configura o wrap do contador PWM para 8 bits (256)
    pwm_set_wrap(slice_num_red, PWM_WRAP);
    pwm_set_wrap(slice_num_blue, PWM_WRAP);
    pwm_set_wrap(slice_num_green, PWM_WRAP);
    pwm_set_wrap(slice_num_buzzer, PWM_WRAP);
    
    // Habilita o PWM
    pwm_set_enabled(slice_num_red, true);
    pwm_set_enabled(slice_num_blue, true);
    pwm_set_enabled(slice_num_green, true);
    pwm_set_enabled(slice_num_buzzer, true);
}

// Tarefa do RGB do semáforo - controla o modo normal e noturno
void vLedRGBNormal(void *pvParameters) {
    initSettings();
    bool modoAtual;

    while (1) {
        // Salva o modo atual para detectar mudanças
        modoAtual = modoNoturno;
        
        if (!modoAtual) {
            // Verde ligado por 4 segundos
            displayState = ESTADO_VERDE;
            configurarLEDs(0, 0, 192);
            
            for (int i = 0; i < 40 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break; // Se o modo mudou, interrompe o ciclo
            }
            
            // Se o modo não mudou durante o delay, vá para a próxima iteração
            if (modoNoturno != modoAtual) continue;
            
            // Amarelo ligado por 2 segundos
            displayState = ESTADO_AMARELO;
            configurarLEDs(192, 8, 100);
            
            for (int i = 0; i < 20 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            
            if (modoNoturno != modoAtual) continue;
            
            // Vermelho ligado por 8 segundos
            displayState = ESTADO_VERMELHO;
            configurarLEDs(192, 0, 0);
            
            for (int i = 0; i < 80 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
        } else {
            // Amarelo ligado por 2 segundos
            displayState = ESTADO_AMARELO;
            configurarLEDs(192, 8, 100);
            
            for (int i = 0; i < 20 && modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            
            if (modoNoturno != modoAtual) continue;
            
            // Todos os LEDs desligados por 2 segundos
            displayState = ESTADO_DESLIGADO;
            configurarLEDs(0, 0, 0);
            
            for (int i = 0; i < 20 && modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
        }
    }
}

// Tarefa do monitoramento do botão - controla o modo normal e noturno
void vBotaoTask(void *pvParameters) {
    const TickType_t tempoDebounce = pdMS_TO_TICKS(200); // 200ms para debounce
    bool leituraAtual;
    
    while (1) {
        // Lê o estado atual do botão
        leituraAtual = gpio_get(BOTAO_A);
        
        // Verifica se o botão foi pressionado
        if (leituraAtual == 0 && estadoBotao == 1) {
            // Verifica se o tempode de debounce foi atingido
            if ((xTaskGetTickCount() - ultimaPressionadoBotao) > tempoDebounce) {
                // Muda o modo
                modoNoturno = !modoNoturno;
                ultimaPressionadoBotao = xTaskGetTickCount();
                printf("Modo alterado para: %s\n", modoNoturno ? "Noturno" : "Normal");
            }
        }
        
        // Atualiza o estado do botão
        estadoBotao = leituraAtual;
        
        // Delay baixo - se for alto trava com freqência
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Tarefa do Buzzer - controla o modo normal e noturno
// A verificação da mudança de modo é feita de maneira semelhante à tarefa do LED RGB
void vBuzzerNormal(void *pvParameters) {
    bool modoAtual;
    
    while (1) {
        // Salva o modo atual para detectar mudanças
        modoAtual = modoNoturno;
        
        if (!modoAtual) {
            // Verde ligado por 4 segundos - 2 ativações buzzer
            configurarBuzzer(5);
            for (int i = 0; i < 10 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) { configurarBuzzer(0); continue; }
            
            configurarBuzzer(0);
            for (int i = 0; i < 10 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) continue;
            
            // Segunda ativação
            configurarBuzzer(5);
            for (int i = 0; i < 10 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) { configurarBuzzer(0); continue; }
            
            configurarBuzzer(0);
            for (int i = 0; i < 10 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) continue;
            
            // Amarelo ligado por 2 segundos - 2 ativações buzzer
            configurarBuzzer(5);
            for (int i = 0; i < 5 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) { configurarBuzzer(0); continue; }
            
            configurarBuzzer(0);
            for (int i = 0; i < 5 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) continue;
            
            // Segunda ativação
            configurarBuzzer(5);
            for (int i = 0; i < 5 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) { configurarBuzzer(0); continue; }
            
            configurarBuzzer(0);
            for (int i = 0; i < 5 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) continue;
            
            // Vermelho ligado por 8 segundos - 4 ativações buzzer
            for (int bipe = 0; bipe < 4 && !modoNoturno; bipe++) {
                configurarBuzzer(5);
                for (int i = 0; i < 5 && !modoNoturno; i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if (modoNoturno != modoAtual) break;
                }
                if (modoNoturno != modoAtual) { configurarBuzzer(0); break; }
                
                configurarBuzzer(0);
                for (int i = 0; i < 15 && !modoNoturno; i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if (modoNoturno != modoAtual) break;
                }
                if (modoNoturno != modoAtual) break;
            }
        } else {
            // Modo noturno
            configurarBuzzer(5);
            for (int i = 0; i < 20 && modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            if (modoNoturno != modoAtual) { configurarBuzzer(0); continue; }
            
            configurarBuzzer(0);
            for (int i = 0; i < 20 && modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
        }
    }
}

// Tarefa da Matriz de LEDs - controla o modo normal e noturno
// A verificação da mudança de modo é feita de maneira semelhante à tarefa do LED RGB
void vLedMatrixNormal(void *pvParameters) {
    state = ESTADO_DESLIGADO; // Inicializa o estado como desligado - refresh display
    npDisplayDigit(state);
    bool modoAtual;

    while (1) {
        // Salva o modo atual para detectar mudanças
        modoAtual = modoNoturno;
        
        if (!modoAtual) {
            // Verde ligado por 4 segundos
            state = ESTADO_VERDE;
            npDisplayDigit(state);
            
            for (int i = 0; i < 40 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            
            if (modoNoturno != modoAtual) continue;
            
            // Amarelo ligado por 2 segundos
            state = ESTADO_AMARELO;
            npDisplayDigit(state);
            
            for (int i = 0; i < 20 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            
            if (modoNoturno != modoAtual) continue;
            
            // Vermelho ligado por 8 segundos
            state = ESTADO_VERMELHO;
            npDisplayDigit(state);
            
            for (int i = 0; i < 80 && !modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
        } else {
            // Amarelo ligado por 2 segundos
            state = ESTADO_AMARELO;
            npDisplayDigit(state);
            
            for (int i = 0; i < 20 && modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
            
            if (modoNoturno != modoAtual) continue;
            
            // Estado desligado por 2 segundos
            state = ESTADO_DESLIGADO;
            npDisplayDigit(state);
            
            for (int i = 0; i < 20 && modoNoturno; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (modoNoturno != modoAtual) break;
            }
        }
    }
}

// Tarefa do display - controla o modo normal e noturno
// A verificação da mudança de modo é feita por meio de variáveis globais que dependem do estado do semáforo
void vDisplayNormal(void *pvParameters) {
    initssd1306();
    int ultimoEstado = -1; // Para detectar mudanças de estado - começa -1 para atualizar pelo menos uma vez

    while (1) {
        // Verifica se houve mudança de estado
        if (ultimoEstado != displayState || ultimoEstado == -1) {
            ultimoEstado = displayState;
            
            // Atualiza o display conforme o estado atual
            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
            ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
            ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
            ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6); // Desenha uma string
            ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);  // Desenha uma string
            ssd1306_draw_string(&ssd, "  FreeRTOS", 10, 28);   // Desenha uma string
            ssd1306_draw_string(&ssd, "Pedreste", 10, 41);       // Desenha uma string
            
            // Mostra "SIGA" apenas quando estiver no estado verde
            if (displayState == ESTADO_VERMELHO) {
                ssd1306_draw_string(&ssd, "SIGA", 10, 50);
            } else {
                ssd1306_draw_string(&ssd, "PARE", 10, 50); // "PARE" - ESTADO VERMELHO
            }
            
            ssd1306_draw_string(&ssd, str_y, 40, 56);          // Desenha uma string
            ssd1306_send_data(&ssd);                           // Atualiza o display
        }
        
        // Se estiver no modo noturno e o display estiver desligado, pisca o display
        if (modoNoturno && displayState == ESTADO_DESLIGADO) {
            ssd1306_fill(&ssd, false);   // Limpa o display (apaga)
            ssd1306_send_data(&ssd);     // Atualiza o display
        }
        
        // Delay pequeno para verificar mudanças de estado
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void initssd1306() {
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Função para configurar o LED na matriz de LEDs
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Função para limpar a matriz de LEDs
void npClear() {
   state = ESTADO_DESLIGADO;
   npDisplayDigit(state);
}

// Inicialização da matriz de LEDs
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, true);
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    npClear();
}

// Função para escrever os dados na matriz de LEDs
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);
}

// Função para obter o índice do LED na matriz de LEDs
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x);
    } else {
        return 24 - (y * 5 + (4 - x));
    }
}

// Função responsável por exibir o dígito na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna);
            npSetLED(
                posicao,
                states[digit][coluna][linha][0],  // R
                states[digit][coluna][linha][1],  // G
                states[digit][coluna][linha][2]   // B
            );
        }
    }
    npWrite();
}