#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <fcntl.h>  // For fcntl, F_SETFL
#include <sys/unistd.h>  // For fileno
#include <sys/stat.h>
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include <fcntl.h>
#include "esp_timer.h"
#include <sys/socket.h>
#include "lwip/netif.h"
#include "lwip/stats.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "esp_private/wifi.h"

#define WIFI_SSID "ESP32_Network"
#define WIFI_PASS "password123"
#define MAX_STA_CONN 4
#define MAX_CLIENTS 8

#define UART_NUM UART_NUM_0
#define BUF_SIZE 128
#define UART_BAUD_RATE 115200

#define GPIO_LEFT 40
#define GPIO_RIGHT 41
#define GPIO_UP 38
#define GPIO_DOWN 39
#define GPIO_PICTURE 42
#define GPIO_VIDEO 45
#define GPIO_MODE 48
#define GPIO_ZOOM_IN 46
#define GPIO_ZOOM_OUT 47
#define GPIO_SPEED_UP 16
#define GPIO_SPEED_DOWN 15
#define GPIO_RASPI_UP 4
#define GPIO_RASPI_DOWN 5
#define GPIO_RASPI_LEFT 6
#define GPIO_RASPI_RIGHT 7
#define GPIO_LED 2

// Command identifiers
#define CMD_UP "up"
#define CMD_DOWN "down"
#define CMD_STEP "step:"    // Format: "step:10" to set step size
#define CMD_LEFT "left"
#define CMD_RIGHT "right"
#define CMD_PICTURE "picture"
#define CMD_VIDEO "video"
#define CMD_ZOOM_IN "zoom-in"
#define CMD_ZOOM_OUT "zoom-out"
#define CMD_START_LEFT "start-left"
#define CMD_STOP_LEFT "stop-left"
#define CMD_START_RIGHT "start-right"
#define CMD_STOP_RIGHT "stop-right"
#define CMD_START_UP "start-up"
#define CMD_STOP_UP "stop-up"
#define CMD_START_DOWN "start-down"
#define CMD_STOP_DOWN "stop-down"
#define CMD_SPEED_UP "speed-up"
#define CMD_SPEED_DOWN "speed-down"

int step = 5;
static const char *TAG = "wifi_ws";
static httpd_handle_t server = NULL;

// Add this after all your #define statements but before app_main
void configure_gpio() {
    // Configure Raspberry Pi input pins with pull-up resistors
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_RASPI_UP) | (1ULL << GPIO_RASPI_DOWN) | 
                           (1ULL << GPIO_RASPI_LEFT) | (1ULL << GPIO_RASPI_RIGHT);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;  // Enable pull-up resistors
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Configure normal button GPIOs
    io_conf.pin_bit_mask = (1ULL << GPIO_LEFT) | (1ULL << GPIO_RIGHT) | 
                           (1ULL << GPIO_UP) | (1ULL << GPIO_DOWN) |
                           (1ULL << GPIO_PICTURE) | (1ULL << GPIO_VIDEO) |
                           (1ULL << GPIO_ZOOM_IN) | (1ULL << GPIO_ZOOM_OUT) |
                           (1ULL << GPIO_MODE) | (1ULL << GPIO_SPEED_DOWN) | (1ULL << GPIO_SPEED_UP);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Configure LED output
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_LED);
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Set initial LED state
    gpio_set_level(GPIO_LED, 0);
    
    ESP_LOGI(TAG, "GPIO configuration complete");
}

// wifi and websockets ----------------------------------------------------------------
// Client tracking structure
typedef struct {
    httpd_handle_t handle;
    int fd;
    bool in_use;
} client_info_t;

static client_info_t clients[MAX_CLIENTS] = {0};

// Function to initialize Wi-Fi in AP mode
static void wifi_init_softap(void) {
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP started, SSID: %s", WIFI_SSID);
}

// Register new client
static void register_client(httpd_handle_t hd, int fd) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i].in_use) {
            clients[i].handle = hd;
            clients[i].fd = fd;
            clients[i].in_use = true;
            ESP_LOGI(TAG, "Client %d registered (fd=%d)", i, fd);
            return;
        }
    }
    ESP_LOGW(TAG, "No free slots for new client");
}

// Unregister client
static void unregister_client(httpd_handle_t hd, int fd) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].in_use && clients[i].fd == fd) {
            clients[i].in_use = false;
            ESP_LOGI(TAG, "Client %d unregistered (fd=%d)", i, fd);
            return;
        }
    }
}

// Function to send a WebSocket command to all clients
static uint64_t total_bytes_sent = 0;
static uint64_t total_bytes_received = 0;

// Call this when you send a WebSocket frame
void log_ws_send(size_t len) {
    total_bytes_sent += len;
}

// Call this when you receive a WebSocket frame
void log_ws_recv(size_t len) {
    total_bytes_received += len;
}
esp_err_t send_websocket_command(const char *command) {
    int clients_count = 0;
    
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t *)command;
    ws_pkt.len = strlen(command);
    
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].in_use) {
            esp_err_t ret = httpd_ws_send_frame_async(clients[i].handle, clients[i].fd, &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send to client %d", i);
                clients[i].in_use = false;
            } else {
                clients_count++;
            }
        }
    }
    log_ws_send(strlen(command));
    if (sizeof(command) >= 50){
        //ESP_LOGI(TAG, "Sent %d bytes of text to %d clients", strlen(command), clients_count);
    } else {
        ESP_LOGI(TAG, "Sent '%s' to %d clients", command, clients_count);
    }
    return (clients_count > 0) ? ESP_OK : ESP_FAIL;
}

// Callback for client connection
static esp_err_t client_connect_handler(httpd_handle_t hd, int fd) {
    ESP_LOGI(TAG, "Client connected (fd=%d)", fd);
    register_client(hd, fd);
    return ESP_OK;
}

// Callback for client disconnection
static void client_disconnect_handler(httpd_handle_t hd, int fd) {
    ESP_LOGI(TAG, "Client disconnected (fd=%d)", fd);
    unregister_client(hd, fd);
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake done");
        return ESP_OK;
    }
    
    // Receive WebSocket data
    uint8_t buf[128] = { 0 };
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive frame");
        return ret;
    }
    
    // Make sure received message is null-terminated
    if (ws_pkt.len < sizeof(buf)) {
        buf[ws_pkt.len] = 0;
    } else {
        buf[sizeof(buf) - 1] = 0;
    }
    
    // Log the received message
    ESP_LOGI(TAG, "Received: %s", ws_pkt.payload);
    
    // Optional: Send acknowledgment
    const char *response = "Message received";
    ws_pkt.payload = (uint8_t*)response;
    ws_pkt.len = strlen(response);
    
    ret = httpd_ws_send_frame(req, &ws_pkt);
    return ret;
}

// HTTP Server Configuration
static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .is_websocket = true
};

// Start WebSocket server
static void start_websocket_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    // Set callback functions
    config.open_fn = client_connect_handler;
    config.close_fn = client_disconnect_handler;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &ws);
        ESP_LOGI(TAG, "WebSocket server started");
    } else {
        ESP_LOGE(TAG, "Failed to start server");
    }
}

// Netowrk Tests
double max_bandwidth_kbps = 0.0;

extern struct netif *esp_netif_get_netif_impl(esp_netif_t *esp_netif);

// static uint32_t last_rx_bytes = 0;
// static uint32_t last_tx_bytes = 0;

// void log_total_wifi_traffic() {
//     esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
//     struct netif *lwip_netif = esp_netif_get_netif_impl(netif);
//     if (lwip_netif == NULL) return;

//     uint32_t rx_now = lwip_netif->mib2_counters.ifin_octets;
//     uint32_t tx_now = lwip_netif->mib2_counters.ifout_octets;

//     ESP_LOGI("NET_STATS", "Wi-Fi AP RX: %u bytes (+%u), TX: %u bytes (+%u)",
//              rx_now, rx_now - last_rx_bytes,
//              tx_now, tx_now - last_tx_bytes);

//     last_rx_bytes = rx_now;
//     last_tx_bytes = tx_now;
// }

#define PORT 5001

void test_network() {
    int listen_sock, sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char rx_buffer[128];
    int len;

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", PORT);

    sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Client connected");

    // === THROUGHPUT MEASUREMENT START ===
    int total_bytes = 0;
    int64_t start_time = esp_timer_get_time(); // microseconds

    while ((len = recv(sock, rx_buffer, sizeof(rx_buffer), 0)) > 0) {
        total_bytes += len;
    }

    int64_t end_time = esp_timer_get_time();
    float seconds = (end_time - start_time) / 1e6f;
    float mb_received = total_bytes / 1e6f;
    float mbps = (total_bytes * 8) / 1e6f / seconds;

    ESP_LOGI(TAG, "Received %.2f MB in %.2f seconds", mb_received, seconds);
    ESP_LOGI(TAG, "Throughput: %.2f Mbps", mbps);
    // === THROUGHPUT MEASUREMENT END ===

    close(sock);
    close(listen_sock);
    ESP_LOGI(TAG, "Connection closed");
    vTaskDelete(NULL);
}

// UART THINGS ----------------------------------------------------------------------------
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

void process_command(const char* cmd) {
    char* tag = "CommandProcessor";
    ESP_LOGI(tag, "Processing command: %s", cmd);

    if (strcmp(cmd, "test_network") == 0) {
        //test_network_bandwidth();
        xTaskCreate(test_network, "test_network", 4096, NULL, 5, NULL);
        return;
    } else if (strcmp(cmd, "network_status") == 0) {
        //log_total_wifi_traffic();
        return;
    }
    
    if (strcmp(cmd, CMD_LEFT) == 0) {
        send_websocket_command(cmd);
        return;
    }
    
    if (strcmp(cmd, CMD_RIGHT) == 0) {
        send_websocket_command(cmd);
        return;
    }

    if (strcmp(cmd, CMD_UP) == 0) {
        send_websocket_command(cmd);
        return;
    }
    
    if (strcmp(cmd, CMD_DOWN) == 0) {
        send_websocket_command(cmd);
        return;
    }

    if (strcmp(cmd, CMD_PICTURE) == 0) {
        send_websocket_command(cmd);
        return;
    }

    if (strcmp(cmd, CMD_VIDEO) == 0) {
        send_websocket_command(cmd);
        return;
    }

    if (strcmp(cmd, CMD_ZOOM_IN) == 0) {
        send_websocket_command(cmd);
        return;
    }

    if (strcmp(cmd, CMD_ZOOM_OUT) == 0) {
        send_websocket_command(cmd);
        return;
    }
    
    // Check for step command
    if (strncmp(cmd, CMD_STEP, strlen(CMD_STEP)) == 0) {
        int new_step = atoi(cmd + strlen(CMD_STEP));
        if (new_step > 0 && new_step <= 300) {
            char message[50];
            sprintf(message, "step:%d", new_step);
            send_websocket_command(message);
        } else {
            printf("Invalid step value (1-300)\r\n");
            ESP_LOGW(tag, "Invalid step value: %d", new_step);
        }
        return;
    }

    ESP_LOGW(tag, "Unknown command: %s", cmd);
    printf("Unknown command\r\n");
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

typedef struct {
    gpio_num_t gpio_normal;        // Main GPIO (always required)
    gpio_num_t gpio_raspi;         // Optional GPIO when mode_gpio is set
    gpio_num_t mode_gpio;          // Optional mode selector GPIO
    const char* cmd_start;         // Required
    const char* cmd_stop;          // Optional
    int prev_state;
    bool use_mode_gpio;            // Set to true if mode_gpio is used
    bool has_stop_cmd;             // Set to true if cmd_stop is provided
} ButtonWatcher;

void update_button(ButtonWatcher* watcher) {
    gpio_num_t active_gpio = watcher->gpio_normal;

    if (watcher->use_mode_gpio) {
        int mode = gpio_get_level(watcher->mode_gpio);
        active_gpio = (mode == 0) ? watcher->gpio_normal : watcher->gpio_raspi;
    }

    int level = gpio_get_level(active_gpio);

    if (level == 0 && watcher->prev_state != 0) {
        int64_t time = esp_timer_get_time();
        ESP_LOGI(TAG, "Button press @Timestamp: %llu us", time);
        send_websocket_command(watcher->cmd_start);
        int64_t after = esp_timer_get_time();
        ESP_LOGI(TAG, "Websocket sent at @Timestamp: %llu us", after);
        int64_t elapsed = (after - time) / 1000;
        ESP_LOGI(TAG, "Time elapsed: %lld ms", elapsed);
        watcher->prev_state = level;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    } else if (watcher->has_stop_cmd && level == 1 && watcher->prev_state != 1) {
        send_websocket_command(watcher->cmd_stop);
        watcher->prev_state = level;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    } else {
        watcher->prev_state = level;
    }
}

// MAIN FUNCTION ---------------------------------------------------------------------

void app_main(void) {
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Start Wi-Fi AP
    wifi_init_softap();

    // Start WebSocket server
    start_websocket_server();

    // Start UART
    usb_cdc_init();
    xTaskCreate(usb_cdc_rx_task, "usb_cdc_rx_task", 4096, NULL, 5, NULL);

    // Configure GPIOs
    configure_gpio();

    ButtonWatcher left_btn = {
        .gpio_normal = GPIO_LEFT,
        .gpio_raspi = GPIO_RASPI_LEFT,
        .mode_gpio = GPIO_MODE,
        .cmd_start = CMD_START_LEFT,
        .cmd_stop = CMD_STOP_LEFT,
        .prev_state = gpio_get_level(GPIO_LEFT),
        .use_mode_gpio = true,
        .has_stop_cmd = true
    };
    ButtonWatcher right_btn = {
        .gpio_normal = GPIO_RIGHT,
        .gpio_raspi = GPIO_RASPI_RIGHT,
        .mode_gpio = GPIO_MODE,
        .cmd_start = CMD_START_RIGHT,
        .cmd_stop = CMD_STOP_RIGHT,
        .prev_state = gpio_get_level(GPIO_RIGHT),
        .use_mode_gpio = true,
        .has_stop_cmd = true
    };
    ButtonWatcher up_btn = {
        .gpio_normal = GPIO_UP,
        .gpio_raspi = GPIO_RASPI_UP,
        .mode_gpio = GPIO_MODE,
        .cmd_start = CMD_START_UP,
        .cmd_stop = CMD_STOP_UP,
        .prev_state = gpio_get_level(GPIO_UP),
        .use_mode_gpio = true,
        .has_stop_cmd = true
    };
    ButtonWatcher down_btn = {
        .gpio_normal = GPIO_DOWN,
        .gpio_raspi = GPIO_RASPI_DOWN,
        .mode_gpio = GPIO_MODE,
        .cmd_start = CMD_START_DOWN,
        .cmd_stop = CMD_STOP_DOWN,
        .prev_state = gpio_get_level(GPIO_DOWN),
        .use_mode_gpio = true,
        .has_stop_cmd = true
    };
    ButtonWatcher picture_btn = {
        .gpio_normal = GPIO_PICTURE,
        .cmd_start = CMD_PICTURE,
        .prev_state = gpio_get_level(GPIO_PICTURE),
        .use_mode_gpio = false,
        .has_stop_cmd = false
    };
    ButtonWatcher video_btn = {
        .gpio_normal = GPIO_VIDEO,
        .cmd_start = CMD_VIDEO,
        .prev_state = gpio_get_level(GPIO_VIDEO),
        .use_mode_gpio = false,
        .has_stop_cmd = false
    };
    ButtonWatcher zoom_in_btn = {
        .gpio_normal = GPIO_ZOOM_IN,
        .cmd_start = CMD_ZOOM_IN,
        .prev_state = gpio_get_level(GPIO_ZOOM_IN),
        .use_mode_gpio = false,
        .has_stop_cmd = false
    };
    ButtonWatcher zoom_out_btn = {
        .gpio_normal = GPIO_ZOOM_OUT,
        .cmd_start = CMD_ZOOM_OUT,
        .prev_state = gpio_get_level(GPIO_ZOOM_OUT),
        .use_mode_gpio = false,
        .has_stop_cmd = false
    };
    ButtonWatcher speed_up_btn = {
        .gpio_normal = GPIO_SPEED_UP,
        .cmd_start = CMD_SPEED_UP,
        .prev_state = gpio_get_level(GPIO_SPEED_UP),
        .use_mode_gpio = false,
        .has_stop_cmd = false
    };
    ButtonWatcher speed_down_btn = {
        .gpio_normal = GPIO_SPEED_DOWN,
        .cmd_start = CMD_SPEED_DOWN,
        .prev_state = gpio_get_level(GPIO_SPEED_DOWN),
        .use_mode_gpio = false,
        .has_stop_cmd = false
    };
    // Debug GPIO state every second
    int debug_counter = 0;
    while (1) {
        update_button(&left_btn);
        update_button(&right_btn);
        update_button(&up_btn);
        update_button(&down_btn);
        update_button(&picture_btn);
        update_button(&video_btn);
        update_button(&zoom_in_btn);
        update_button(&zoom_out_btn);
        update_button(&speed_up_btn);
        update_button(&speed_down_btn);
        
        // Print GPIO debug info every second
        if (debug_counter >= 500) {
            ESP_LOGI(TAG, "RasPi GPIO Levels - LEFT: %d, RIGHT: %d, UP: %d, DOWN: %d", 
                gpio_get_level(GPIO_RASPI_LEFT),
                gpio_get_level(GPIO_RASPI_RIGHT),
                gpio_get_level(GPIO_RASPI_UP),
                gpio_get_level(GPIO_RASPI_DOWN));
            debug_counter = 0;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
        //debug_counter++;
    }
}