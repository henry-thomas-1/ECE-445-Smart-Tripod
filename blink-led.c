#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_websocket_client.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"

#define WIFI_SSID "ESP32_Network"
#define WIFI_PASS "password123"
#define WEBSOCKET_URI "ws://192.168.4.1/ws"
static const char *TAG = "wifi_ws_client";
static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static uint32_t last_print_time = 0;
char IP_ADDRESS[16] = {0};

#define BLINK_LED 5
#define SERVO_PIN_1 4
#define SERVO_PIN_2 5

#define UART_NUM UART_NUM_0
#define BUF_SIZE 128
#define UART_BAUD_RATE 115200

// Command identifiers
#define CMD_UP "up"
#define CMD_DOWN "down"
#define CMD_STEP "step:"    // Format: "step:10" to set step size
#define CMD_LEFT "left"
#define CMD_RIGHT "right"
#define CMD_CENTER "center"
#define CMD_START_LEFT "start-left"
#define CMD_STOP_LEFT "stop-left"
#define CMD_START_RIGHT "start-right"
#define CMD_STOP_RIGHT "stop-right"
#define CMD_START_UP "start-up"
#define CMD_STOP_UP "stop-up"
#define CMD_START_DOWN "start-down"
#define CMD_STOP_DOWN "stop-down"
#define CMD_NETWORK "test_network"
#define CMD_SPEED_UP "speed-up"
#define CMD_SPEED_DOWN "speed-down"

// Global parameters
static int g_step = 5;              // Default step size
static int g_current_duty_0 = 500;    // Keep track of current duty cycle
static int g_current_duty_1 = 1500;

static int turning_left_state = 0;
static int turning_right_state = 0;
static int turning_up_state = 0;
static int turning_down_state = 0;

// UART buffer
static QueueHandle_t uart_queue;

void uart_init() {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Function to update servo position
void update_servo_position(int duty, int channel) {
    char* tag = "ServoControl";
    
    int min_duty, max_duty;
    
    // Set min and max duty cycle based on the channel
    if (channel == 0) {
        min_duty = 300;
        max_duty = 1000;
    } else if (channel == 1) {
        min_duty = 700;
        max_duty = 2300;
    } else {
        // Default values if unknown channel (optional safeguard)
        min_duty = 500;
        max_duty = 1000;
    }
    
    // Limit duty cycle to safe values
    if (duty < min_duty) duty = min_duty;
    if (duty > max_duty) duty = max_duty;
    
    // Save the current duty cycle
    if (channel == 0){
        g_current_duty_0 = duty;
    } else {
        g_current_duty_1 = duty;
    }
    
    // Update the servo position
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    
    ESP_LOGI(tag, "Servo %d position updated to duty: %d", channel, duty);
}


void turning_left(){
    while (turning_left_state == 1){
        update_servo_position(g_current_duty_1 + g_step, LEDC_CHANNEL_1);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void turning_right(){
    while (turning_right_state == 1){
        update_servo_position(g_current_duty_1 - g_step, LEDC_CHANNEL_1);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void turning_up(){
    while (turning_up_state == 1){
        update_servo_position(g_current_duty_0 - g_step, LEDC_CHANNEL_0);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "duty cycle: %d", g_current_duty_0);
    }
    vTaskDelete(NULL);
}

void turning_down(){
    while (turning_down_state == 1){
        update_servo_position(g_current_duty_0 + g_step, LEDC_CHANNEL_0);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "duty cycle: %d", g_current_duty_0);
    }
    vTaskDelete(NULL);
}
void usb_cdc_init(void) {
    // Initialize USB CDC
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));

    // Redirect console output to USB CDC
    usb_serial_jtag_vfs_use_driver();
    
    // Set stdin non-blocking
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);
}

void test_network(void) {
    ESP_LOGI(TAG, "IP Adress: %s", IP_ADDRESS);
}

void process_command(const char* cmd) {
    char* tag = "CommandProcessor";
    ESP_LOGI(tag, "Processing command: %s", cmd);
    
    // Check for left command - decrease duty cycle to turn left
    if (strcmp(cmd, CMD_LEFT) == 0) {
        update_servo_position(g_current_duty_1 - g_step, LEDC_CHANNEL_1);  // Move left by step amount
        char response[50];
        sprintf(response, "Turned left, duty: %d\r\n", g_current_duty_1);
        uart_write_bytes(UART_NUM, response, strlen(response));
        return;
    }
    
    // Check for right command - increase duty cycle to turn right
    if (strcmp(cmd, CMD_RIGHT) == 0) {
        update_servo_position(g_current_duty_1 + g_step, LEDC_CHANNEL_1);  // Move right by step amount
        char response[50];
        sprintf(response, "Turned right, duty: %d\r\n", g_current_duty_1);
        uart_write_bytes(UART_NUM, response, strlen(response));
        return;
    }

    // Check for up command - decrease duty cycle to turn left
    if (strcmp(cmd, CMD_UP) == 0) {
        update_servo_position(g_current_duty_0 - g_step, LEDC_CHANNEL_0);  // Move left by step amount
        char response[50];
        sprintf(response, "Turned up, duty: %d\r\n", g_current_duty_0);
        uart_write_bytes(UART_NUM, response, strlen(response));
        return;
    }
    
    // Check for down command - increase duty cycle to turn right
    if (strcmp(cmd, CMD_DOWN) == 0) {
        update_servo_position(g_current_duty_0 + g_step, LEDC_CHANNEL_0);  // Move right by step amount
        char response[50];
        sprintf(response, "Turned down, duty: %d\r\n", g_current_duty_0);
        uart_write_bytes(UART_NUM, response, strlen(response));
        return;
    }

    if (strcmp(cmd, CMD_START_LEFT) == 0 && turning_left_state == 0) {
        int64_t time = esp_timer_get_time();
        ESP_LOGI(tag, "Recieved command @Timestamp: %llu us", time);
        turning_left_state = 1;
        xTaskCreate(turning_left, "turning_left", 4096, NULL, 5, NULL);
        int64_t after = esp_timer_get_time();
        int64_t elapsed = (after - time) / 1000;
        ESP_LOGI(tag, "Elapsed time: %lld ms", elapsed);
        return;
    }
    if (strcmp(cmd, CMD_START_RIGHT) == 0 && turning_right_state == 0) {
        turning_right_state = 1;
        xTaskCreate(turning_right, "turning_right", 4096, NULL, 5, NULL);
        return;
    }
    if (strcmp(cmd, CMD_START_UP) == 0 && turning_up_state == 0) {
        turning_up_state = 1;
        xTaskCreate(turning_up, "turning_up", 4096, NULL, 5, NULL);
        return;
    }
    if (strcmp(cmd, CMD_START_DOWN) == 0 && turning_down_state == 0) {
        turning_down_state = 1;
        xTaskCreate(turning_down, "turning_down", 4096, NULL, 5, NULL);
        return;
    }
    if (strcmp(cmd, CMD_STOP_LEFT) == 0) {
        turning_left_state = 0;
        return;
    }
    if (strcmp(cmd, CMD_STOP_RIGHT) == 0) {
        turning_right_state = 0;
        return;
    }
    if (strcmp(cmd, CMD_STOP_UP) == 0) {
        turning_up_state = 0;
        return;
    }
    if (strcmp(cmd, CMD_STOP_DOWN) == 0) {
        turning_down_state = 0;
        return;
    }
    if (strcmp(cmd, CMD_NETWORK) == 0) {
        test_network();
        return;
    }
    if (strcmp(cmd, CMD_SPEED_UP) == 0) {
        if (g_step >= 15){
            ESP_LOGI(tag, "Maximum speed already reached");
        }else {
            g_step++;
            ESP_LOGI(tag, "Step size set to %d", g_step);
        }
        return;
    }
    if (strcmp(cmd, CMD_SPEED_DOWN) == 0) {
        if (g_step <= 1){
            ESP_LOGI(tag, "Minimum speed already reached");
        }else {
            g_step--;
            ESP_LOGI(tag, "Step size set to %d", g_step);
        }
        return;
    }

    
    // Check for step command
    if (strncmp(cmd, CMD_STEP, strlen(CMD_STEP)) == 0) {
        int new_step = atoi(cmd + strlen(CMD_STEP));
        if (new_step > 0 && new_step <= 300) {
            g_step = new_step;
            char response[50];
            sprintf(response, "Step size set to %d\r\n", g_step);
            uart_write_bytes(UART_NUM, response, strlen(response));
            ESP_LOGI(tag, "Step size set to %d", g_step);
        } else {
            uart_write_bytes(UART_NUM, "Invalid step value (1-300)\r\n", 27);
            ESP_LOGW(tag, "Invalid step value: %d", new_step);
        }
        return;
    }

    if (strncmp(cmd, CMD_CENTER, strlen(CMD_CENTER)) == 0) {
        update_servo_position(500, LEDC_CHANNEL_0);
        update_servo_position(1500, LEDC_CHANNEL_1);
    }

    // Unknown command
    ESP_LOGW(tag, "Unknown command: %s", cmd);
    uart_write_bytes(UART_NUM, "Unknown command\r\n", 17);
}

void usb_cdc_rx_task(void *pvParameters) {
    // Initial message using direct USB Serial JTAG function
    const char *message = "USB CDC Ready. Type commands and press Enter:\r\n";
    usb_serial_jtag_write_bytes((const uint8_t*)message, strlen(message), portMAX_DELAY);

    char input[128];
    int input_pos = 0;
    uint8_t data;
    int len;
    
    while (1) {
        // Read one byte at a time using direct USB Serial JTAG function
        len = usb_serial_jtag_read_bytes(&data, 1, 0);
        
        if (len > 0) {
            // Echo character back
            usb_serial_jtag_write_bytes(&data, 1, portMAX_DELAY);
            
            if (data == '\n' || data == '\r') {
                // End of command
                if (input_pos > 0) {
                    input[input_pos] = '\0';
                    
                    // Send processing message
                    char temp[150];
                    snprintf(temp, sizeof(temp), "\r\nProcessing: %s\r\n", input);
                    usb_serial_jtag_write_bytes((const uint8_t*)temp, strlen(temp), portMAX_DELAY);
                    
                    // Process the command
                    process_command(input);
                    input_pos = 0;
                }
            } else if (input_pos < sizeof(input) - 1) {
                // Add to buffer
                input[input_pos++] = (char)data;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void init_servo(void) {
    char* tag = "ServoInit";
    ESP_LOGI(tag, "Initializing servo");

    ledc_timer_config_t timer_conf_0 = {
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t timer_conf_1 = {
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf_0);
    ledc_timer_config(&timer_conf_1);

    ledc_channel_config_t ledc_conf_0 = {
        .channel = LEDC_CHANNEL_0,
        .duty = g_current_duty_0,
        .gpio_num = SERVO_PIN_1,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config_t ledc_conf_1 = {
        .channel = LEDC_CHANNEL_1,
        .duty = g_current_duty_1,
        .gpio_num = SERVO_PIN_2,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1
    };
    ledc_channel_config(&ledc_conf_0);
    ledc_channel_config(&ledc_conf_1);
    
    // Set the initial position
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, g_current_duty_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(tag, "Servo %d initialized at duty: %d", LEDC_CHANNEL_0, g_current_duty_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g_current_duty_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ESP_LOGI(tag, "Servo %d initialized at duty: %d", LEDC_CHANNEL_1, g_current_duty_1);
}

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, 
                              int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "Connecting to AP...");
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGI(TAG, "Disconnected from AP, retrying...");
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
        esp_ip4addr_ntoa(&event->ip_info.ip, IP_ADDRESS, sizeof(IP_ADDRESS));
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

// Initialize WiFi in station mode
static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "wifi_init_sta finished");
}

// WebSocket event handler
static void websocket_event_handler(void *handler_args, esp_event_base_t base, 
                                   int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            break;
            
        case WEBSOCKET_EVENT_DATA:
            // Handle received data
            if (data->data_len > 0) {
                // Create a null-terminated string from the received data
                char *command = malloc(data->data_len + 1);
                if (command) {
                    memcpy(command, data->data_ptr, data->data_len);
                    command[data->data_len] = 0;
                    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (current_time - last_print_time > 200) { // 500ms rate limiting
                    ESP_LOGI(TAG, "Received command: %s", command);
                    last_print_time = current_time;
                }
                    ESP_LOGI(TAG, "Received command: %s", command);
                    process_command(command);
                    free(command);
                }
            }
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
            break;
    }
}

void app_main(void) {
    char* tag = pcTaskGetName(NULL);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Wait for WiFi connection
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    
    // Configure WebSocket client
    esp_websocket_client_config_t websocket_cfg = {
        .uri = WEBSOCKET_URI,
        .reconnect_timeout_ms = 10000,
        .task_stack = 4096,
    };
    
    // Initialize WebSocket client
    esp_websocket_client_handle_t client = esp_websocket_client_init(&websocket_cfg);
    
    // Register WebSocket events
    ESP_ERROR_CHECK(esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL));
    
    // Start WebSocket client
    esp_websocket_client_start(client);
    ESP_LOGI(TAG, "WebSocket client started");

    ESP_LOGI(tag, "Starting up servo manual control with UART commands");

    // Initialize GPIO for LED
    gpio_reset_pin(BLINK_LED);
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
    
    // Initialize servo
    init_servo();
    
    // Initialize UART
    uart_init();
    
    // Create UART task
    usb_cdc_init();
    xTaskCreate(usb_cdc_rx_task, "usb_cdc_rx_task", 4096, NULL, 5, NULL);
    // Heartbeat LED in main task
    while (1) {
        gpio_set_level(BLINK_LED, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_LED, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}