#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <esp/gpio.h>

//Wifi
#include <wifi_config.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

//Mqtt
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <semphr.h>

//Mqtt credentials
#include <mqtt_config.h>

#define TRIGGER_GPIO 4
#define ECHO_GPIO    5
#define LED_GPIO     2  // GPIO2 corresponds to the built-in LED on most ESP8266 boards

#define TRIGGER_DELAY_US 2
#define TRIGGER_PULSE_WIDTH_US 10

SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

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

//MQTT task
/*static void mqqt_task(void * pvParameters)
{
    int ret = 0;
    struct mqtt_network mqtt_network;
    mqtt_client_t client   = mqtt_client_default;

}*/

static void wifi_task(void * pvParameters)
{
    uint8_t status = 0;
    uint8_t retries = 10;

    struct sdk_station_config config = {
        .ssid = SSID,
        .password = PASSWORD
    };
    
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status );
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n\r");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
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

    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    //xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);

    // Create the ultrasonic sensor task
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, NULL, 2, NULL);
}
