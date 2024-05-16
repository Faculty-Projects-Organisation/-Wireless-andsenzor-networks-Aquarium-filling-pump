#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "i2c/i2c.h"
#include <stdio.h>
#include <string.h>
#include "task.h"

#define US100_TX    4     // GPIO4 for TX (connect to RX pin of US-100)
#define US100_RX    5     // GPIO5 for RX (connect to TX pin of US-100)
#define BAUD_RATE   9600  // Baud rate for UART communication

void uart_write_bytes(const uint8_t *data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uart_putc(0, data[i]);
    }
}

void uart_read_bytes(uint8_t *data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        data[i] = uart_getc(0);
    }
}

void ultrasonic_task(void *pvParameters)
{
    uint8_t txData = 0x55;
    uint8_t rxBuffer[2];

    while (1) {
        // Send command
        uart_write_bytes(&txData, 1);

        vTaskDelay(pdMS_TO_TICKS(100));

        // Read response (two bytes)
        uart_read_bytes(rxBuffer, 2);

        // Combine the received bytes into a 16-bit value (assuming little-endian)
        uint16_t distance = (rxBuffer[1] << 8) | rxBuffer[0];

        if (distance < 10000 || distance > 1) {
            printf("Distance: %d mm\n", distance);
        } else {
            printf("Measuring");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void user_init(void)
{

    uart_set_baud(0, BAUD_RATE);


    i2c_init(0, US100_TX, US100_RX, I2C_FREQ_100K);

    gpio_enable(US100_TX, GPIO_OUTPUT); // Set TX pin as output
    gpio_enable(US100_RX, GPIO_INPUT);  // Set RX pin as input

    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, NULL, 2, NULL);
    
}
