#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <esp/gpio.h>
#include <wifi_config.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>
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


//VARIABLES FOR RECEIVED TOPICS
uint8_t fill_value;
uint8_t auto_value;
uint8_t manual_value;
uint8_t max_value;
uint8_t min_value;
// Mutex to protect access to these variables
SemaphoreHandle_t mutex;


#define PUB_MSG_LEN 16
#define TOPIC_LEN 32

typedef struct {
    char topic[TOPIC_LEN];
    union {
        float float_val;
        uint8_t byte_val;
        char msg[PUB_MSG_LEN];
    } payload;
    uint8_t payload_type; // 0 for float, 1 for byte
    uint8_t type_of_measurement; // 0 for waterLevel, 1 temperature
} publish_message_t;

static void topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;
    char topic_name[md->topic->lenstring.len + 1];
    memcpy(topic_name, md->topic->lenstring.data, md->topic->lenstring.len);
    topic_name[md->topic->lenstring.len] = '\0';  // Null-terminate the topic name

    printf("Received: %s = ", topic_name);
    for (i = 0; i < (int)message->payloadlen; ++i)
        printf("%c", ((char *)(message->payload))[i]);
    printf("\r\n");

    // Save the received value based on the topic
    if (message->payloadlen == 1) { // Byte type
        uint8_t received_value = *((uint8_t *)(message->payload));

        xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before modifying shared variables

        if (strcmp(topic_name, "/NODE03/fill") == 0) {
            fill_value = received_value;
        } else if (strcmp(topic_name, "/NODE03/auto") == 0) {
            auto_value = received_value;
        } else if (strcmp(topic_name, "/NODE03/manual") == 0) {
            manual_value = received_value;
        } else if (strcmp(topic_name, "/NODE03/max") == 0) {
            max_value = received_value;
        } else if (strcmp(topic_name, "/NODE03/min") == 0) {
            min_value = received_value;
        }

        xSemaphoreGive(mutex); // Release the mutex after modifying shared variables
    }
}


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

static void mqtt_task(void *pvParameters)
{
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new(&network);
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while (1)
    {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ", __func__, MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if (ret)
        {
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);

        data.willFlag = 0;
        data.MQTTVersion = 3;
        data.clientID.cstring = mqtt_client_id;
        data.username.cstring = MQTT_USER;
        data.password.cstring = MQTT_PASS;
        data.keepAliveInterval = 10;
        data.cleansession = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if (ret)
        {
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\r\n");
        mqtt_subscribe(&client, "/NODE03/fill", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/auto", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/manual", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/max", MQTT_QOS1, topic_received);
        mqtt_subscribe(&client, "/NODE03/min", MQTT_QOS1, topic_received);
        xQueueReset(publish_queue);

        while (1)
        {
            char msg[PUB_MSG_LEN - 1] = "\0";
            while (xQueueReceive(publish_queue, (void *)msg, 0) == pdTRUE)
            {
                printf("got message to publish\r\n");
                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;

                // Publish to /NODE03/waterLevel
                ret = mqtt_publish(&client, "/NODE03/waterLevel", &message);
                if (ret != MQTT_SUCCESS)
                {
                    printf("error while publishing message: %d\n", ret);
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

// OK.
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

// Function to get unique ID based on MAC address
static const char *get_my_id(void)
{
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

        publish_message_t wL;
        wL.payload.float_val = (float) distance_cm;
        wL.payload_type = 0;
        Wl.payload.type_of_measurement = 0;

        if(xQueueSend(publish_queue, &wL, 0) == pdFALSE){
            printf("PQ owerflow");
        }
       
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
    xTaskCreate(&wifi_task, "wifi_task", 256, NULL, 2, NULL);
    // xTaskCreate(&beat_task, "beat_task", 256, NULL, 3, NULL);
    xTaskCreate(&ultrasonic_task, "ultrasonic_task", 256, NULL, 3, NULL); // Added ultrasonic task
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
}
