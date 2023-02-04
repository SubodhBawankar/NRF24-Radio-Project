// Standard Library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ESP-IDF Library
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/adc.h"

// Extra Libraries
#include "NRF24L01.h"

static const char* TAG = "Chat";




void Reciever(){
    NRF24_t device;
    // device = {
    //     .cePin = 16;
    //     .csnPin = 17;
    //     .channel = 50;
    // }
    device.cePin = 16;
    device.csnPin = 17;
    device.channel = 50;
    SPI_Config(&device);
    Register_Config(&device , 50, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}


void app_main(void)
{
    printf("Welcome");
    xTaskCreate(
                Reciever,
                "Reciever",
                1024*3,
                NULL,
                1,
                NULL
            );
}
