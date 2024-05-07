#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <esp/gpio.h>

#define TRIGGER_GPIO 4
#define ECHO_GPIO    5
#define LED_GPIO     2  // GPIO2 corresponds to the built-in LED on most ESP8266 boards

#define TRIGGER_DELAY_US 2
#define TRIGGER_PULSE_WIDTH_US 10

uint32_t pulseIn(uint32_t gpio_num, bool level, uint32_t timeout_us)
{
    uint32_t start_time = sdk_system_get_time();

    while (sdk_system_get_time() - start_time < timeout_us)
    {
        if (gpio_read(gpio_num) == level)
        {
            uint32_t pulse_start = sdk_system_get_time();
            while (gpio_read(gpio_num) == level)
            {
                if (sdk_system_get_time() - pulse_start > timeout_us)
                    return 0;
            }
            return sdk_system_get_time() - pulse_start;
        }
    }

    return 0;
}

void ultrasonic_task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        gpio_write(TRIGGER_GPIO, false);  // Set trigger pin LOW
        vTaskDelay(pdMS_TO_TICKS(2));     // Wait for at least 2 milliseconds

        gpio_write(TRIGGER_GPIO, true);   // Set trigger pin HIGH
        vTaskDelay(pdMS_TO_TICKS(10));    // Keep trigger pin HIGH for 10 microseconds

        gpio_write(TRIGGER_GPIO, false);  // Set trigger pin LOW

        // Measure the duration of the HIGH pulse on the ECHO pin
        uint32_t duration = pulseIn(ECHO_GPIO, true, 30000); // Max timeout of 30 milliseconds

        // Calculate distance in centimeters
        uint32_t distance_cm = (duration * 34) / (2 * 1000); // Speed of sound is approximately 34 microseconds per centimeter

        // Print distance to serial console
        printf("Centimeter: %u\n", distance_cm);

        // Toggle LED based on distance (optional)
        gpio_toggle(LED_GPIO);

        // Delay before the next measurement
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000)); // Delay for 1000 milliseconds
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200); // Set UART baud rate to 115200

    // Configure GPIO pins
    gpio_enable(TRIGGER_GPIO, GPIO_OUTPUT);
    gpio_enable(ECHO_GPIO, GPIO_INPUT);
    gpio_enable(LED_GPIO, GPIO_OUTPUT);

    // Create the ultrasonic sensor task
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, NULL, 2, NULL);
}
