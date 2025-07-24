#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
 #include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"


#define led1 2

uint8_t led_level = 0;
static const char *tag = "Main";
TimerHandle_t xTimers;
int interval = 1000;
int timerID = 1;

// Declaracion  de funciones
esp_err_t init_led(void);
esp_err_t blink_led(void);
esp_err_t set_timer(void);

//  Callback del temporizador
void vTimerCallback(TimerHandle_t xTimer)
{
    blink_led();
    ESP_LOGI(tag, "Temporizador activado, LED cambiado de estado");
}

// Función principal
void app_main(void)
{
    init_led();
    set_timer();

     
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}

 
esp_err_t init_led(void)
{
    gpio_reset_pin(led1);
    gpio_set_direction(led1, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

 
esp_err_t blink_led(void)
{
    led_level = !led_level;
    gpio_set_level(led1, led_level);
    return ESP_OK;
}

 
esp_err_t set_timer(void)
{
    ESP_LOGI(tag, "Configuración inicial del temporizador");
    xTimers = xTimerCreate("Timer",              // Nombre de  temporizador
                           pdMS_TO_TICKS(interval), // Período de 1 seg 
                           pdTRUE,               // Recarga automática
                           (void *)timerID,      // ID -OPCIONAL-
                           vTimerCallback);      //  callback

    if (xTimers == NULL)
    {
        ESP_LOGE(tag, "Error al crear temporizador");
        return ESP_FAIL;
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            ESP_LOGE(tag, "Error al iniciar temporizador");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}
