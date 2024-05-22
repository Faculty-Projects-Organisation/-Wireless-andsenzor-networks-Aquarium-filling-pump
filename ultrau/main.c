#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "i2c/i2c.h"
#include <stdio.h>
#include <string.h>
#include "task.h"

#define US100_TX 4
#define US100_RX 5
#define BAUD_RATE 9600

void uart_write_bytes(const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        uart_putc(0, data[i]);
    }
}

void uart_read_bytes(uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        data[i] = uart_getc(0);
    }
}

void ultrasonic_task(void *pvParameters)
{
    uint8_t txData = 0x55;
    uint8_t rxBuffer[2];

    uint8_t txTemp = 0x50;
    uint8_t rxTBuffer[1];

    while (1)
    {
        // distance
        uart_write_bytes(&txData, 1);

        vTaskDelay(pdMS_TO_TICKS(100));

        uart_read_bytes(rxBuffer, 2);

        uint16_t distance = (rxBuffer[1] << 8) | rxBuffer[0];

        if (distance < 10000 || distance > 1)
        {
            printf("Distance: %d mm\n", distance);
        }
        else
        {
            printf("Measuring");
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        // temperature
        uart_write_bytes(&txTemp, 1);

        vTaskDelay(pdMS_TO_TICKS(50));

        uart_read_bytes(rxTBuffer, 1);

        uint16_t temperature = rxTBuffer[0];

        if ((temperature > 1) && (temperature < 130))
        {
            temperature -= 45;
            printf("Temperature: %d degrees\n", (temperature));
        }
        else
        {
            printf("Invalid temperature: %d\n", temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

void user_init(void)
{

    uart_set_baud(0, BAUD_RATE);

    i2c_init(0, US100_TX, US100_RX, I2C_FREQ_100K);

    gpio_enable(US100_TX, GPIO_OUTPUT);
    gpio_enable(US100_RX, GPIO_INPUT);

    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, NULL, 2, NULL);
}
