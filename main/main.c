/**
 * Utilizado um ESP32 para fazer a comunicação com outro ESP32 por protocolo proprietário 3W2B
 * Protocolo serial - Sinais de clk, sync, data
 * Envia 2 byte por vez - 16 bits
 * 
 * Este é o programa do ESP32 que recebe os dados
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FREERTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

gpio_num_t CLK = GPIO_NUM_23;
gpio_num_t SYNC = GPIO_NUM_22;
gpio_num_t DATA = GPIO_NUM_21;

SemaphoreHandle_t smf_clk;

uint16_t buffer_3w2b = 0, 
         aux         = 0;
char     flag_sync   = 0;

void IRAM_ATTR isr_clk(void *arg)
{
    BaseType_t aux = false;
    xSemaphoreGiveFromISR(smf_clk, &aux);

    buffer_3w2b <<= 1;                                  // Desloca para esquerda 1 bit 
    buffer_3w2b |= (gpio_get_level(DATA) & 0x0001);     // Leitura apenas do bit menos significativo
}

void task_3w2b(void *arg)
{
    while(true)
    {
        if(xSemaphoreTake(smf_clk, pdMS_TO_TICKS(10) == true))
        {
            // buffer_3w2b <<= 1;                                  // Desloca para esquerda 1 bit 
            // buffer_3w2b |= (gpio_get_level(DATA) & 0x0001);     // Leitura apenas do bit menos significativo
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void gpio_cfg(gpio_num_t GPIO_NUM, gpio_isr_t ISR, gpio_mode_t MODE, gpio_int_type_t INTR_TYPE)
{
    gpio_config_t gpio_cfg = 
    {
        .pin_bit_mask = (1 << GPIO_NUM),
        .mode = MODE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = INTR_TYPE
    };

    gpio_intr_enable(GPIO_NUM);
    gpio_isr_handler_add(GPIO_NUM, ISR, (void*)GPIO_NUM);

    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
}

void app_main(void)
{
    /* Inicializa o semaphoro */
    smf_clk = xSemaphoreCreateBinary(); 

    /* Instalação do serviço de isr para as interrupções */
    gpio_install_isr_service(0);
    
    /* Configura os GPIOs */
    gpio_cfg(CLK, isr_clk, GPIO_MODE_INPUT, GPIO_INTR_POSEDGE);
    gpio_cfg(SYNC, NULL, GPIO_MODE_INPUT, GPIO_INTR_DISABLE);
    gpio_cfg(DATA, NULL, GPIO_MODE_INPUT, GPIO_INTR_DISABLE);

    /* Interrupção recebimento de dados */
    xTaskCreatePinnedToCore(task_3w2b, "task_3w2b", 4096, NULL, 2, NULL, 0);  

    while(true)
    {
        if(!gpio_get_level(SYNC))
        {
            flag_sync = 1;
        }

        if(gpio_get_level(SYNC) && flag_sync)
        {
            aux = buffer_3w2b;
            buffer_3w2b = 0;
            flag_sync = 0;
            ESP_LOGI("MAIN", "BUFFER = %d", aux);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
