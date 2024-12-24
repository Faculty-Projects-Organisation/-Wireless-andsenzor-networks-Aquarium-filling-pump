#include "espressif/esp_common.h"
#include "esp/uart.h"
#include <FreeRTOS.h>
#include <i2c/i2c.h>
#include <stdio.h>
#include <string.h>
#include <task.h>

// WiFi
#include <wifi_config.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

// MQTT
#include <mqtt_config.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <semphr.h>

//SENSOR PIN DEFINITIONS
#define US100_TX 4
#define US100_RX 5
#define BAUD_RATE 9600

// Globals

#define PUB_MSG_LEN 64
QueueHandle_t publish_queue;



//SENSOR FUNCTIONS
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

        char msg[PUB_MSG_LEN];
        snprintf(msg, PUB_MSG_LEN, "%d", distance);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE)
        {
            printf("Publish queue overflow.\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds

        // Temperature
        /*
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
        }*/

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

//MQTT and WiFI
typedef struct 
{
    int fill;
    int auto_mode;
    int manual;
    int max;
    int min;
} TopicValues;

TopicValues topic_values = {0};
SemaphoreHandle_t topic_values_mutex;

SemaphoreHandle_t wifi_alive;





/*static void  beat_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];
    int count = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10000 / portTICK_PERIOD_MS);
        printf("beat\r\n");
        snprintf(msg, PUB_MSG_LEN, "Beat %d\r\n", count++);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
            printf("Publish queue overflow.\r\n");
        }
    }
}*/

static void  topic_received(mqtt_message_data_t *md)
{
    mqtt_message_t *message = md->message;
    char topic[md->topic->lenstring.len + 1];
    char payload[message->payloadlen + 1];

    strncpy(topic, md->topic->lenstring.data, md->topic->lenstring.len);
    topic[md->topic->lenstring.len] = '\0';
    strncpy(payload, (char *)message->payload, message->payloadlen);
    payload[message->payloadlen] = '\0';

    int value = atoi(payload);

    xSemaphoreTake(topic_values_mutex, portMAX_DELAY);
    if (strcmp(topic, "/NODE03/fill") == 0)
    {
        topic_values.fill = value;
    }
    else if (strcmp(topic, "/NODE03/auto") == 0)
    {
        topic_values.auto_mode = value;
    }
    else if (strcmp(topic, "/NODE03/manual") == 0)
    {
        topic_values.manual = value;
    }
    else if (strcmp(topic, "/NODE03/max") == 0)
    {
        topic_values.max = value;
    }
    else if (strcmp(topic, "/NODE03/min") == 0)
    {
        topic_values.min = value;
    }
    xSemaphoreGive(topic_values_mutex);

    printf("Received topic: %s, value: %d\n", topic, value);
}


static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static void  mqtt_task(void *pvParameters)
{
    int ret         = 0;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100,
                      mqtt_readbuf, 100);

        data.willFlag       = 0;
        data.MQTTVersion    = 3;
        data.clientID.cstring   = mqtt_client_id;
        data.username.cstring   = MQTT_USER;
        data.password.cstring   = MQTT_PASS;
        data.keepAliveInterval  = 10;
        data.cleansession   = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        
        mqtt_subscribe(&client, "/NODE03/fill", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/auto", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/manual", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/max", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/min", MQTT_QOS1, topic_received);
        xQueueReset(publish_queue);

        while(1){

            char msg[PUB_MSG_LEN - 1] = "\0";
            while(xQueueReceive(publish_queue, (void *)msg, 0) ==
                  pdTRUE){

                mqtt_message_t message;
                message.payload = (void *)msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, "/NODE03/data", &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}

static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n\r");
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

void user_init(void)
{
    uart_set_baud(0, BAUD_RATE);
    i2c_init(0, US100_TX, US100_RX, I2C_FREQ_100K);

    gpio_enable(US100_TX, GPIO_OUTPUT);
    gpio_enable(US100_RX, GPIO_OUTPUT);

    vSemaphoreCreateBinary(wifi_alive);
    vSemaphoreCreateBinary(topic_values_mutex);
    publish_queue = xQueueCreate(3, 64);

    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, NULL, 2, NULL);
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    //xTaskCreate(&beat_task, "beat_task", 256, NULL, 3, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
}