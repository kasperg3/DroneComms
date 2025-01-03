#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <cstring>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "driver/uart.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include <string>
#include "driver/gpio.h"
#define TAG "ESP-NOW-UART"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "tinyusb.h"
#include "lwip/sockets.h"

#define UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_21)
#define RXD_PIN (GPIO_NUM_20)
#define UART_BUF_SIZE (1024)
// Data structure for ESP-NOW
struct Message
{
    uint8_t macAddress[6];
    float value;
};

// Queue handle for ESP-NOW and UART
QueueHandle_t espNowQueue;
QueueHandle_t uartQueue;

// Define ESPNowHandler class
class ESPNowHandler
{
public:
    ESPNowHandler()
    {
        esp_err_t ret;

        // Initialize NVS
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        // Initialize Wi-Fi in station mode
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        // Initialize ESP-NOW
        ret = esp_now_init();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "ESP-NOW initialization failed: %s", esp_err_to_name(ret));
            abort();
        }

        // Register callbacks
        esp_now_register_recv_cb(onDataReceive);
        esp_now_register_send_cb(onDataSend);

        // Add a broadcast peer
        esp_now_peer_info_t peerInfo{};
        std::memset(peerInfo.peer_addr, 0xFF, ESP_NOW_ETH_ALEN); // Broadcast address
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        ret = esp_now_add_peer(&peerInfo);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(ret));
            abort();
        }
    }

    static void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
    {
        Message receivedData;
        std::memcpy(&receivedData, data, sizeof(receivedData));
        ESP_LOGI(TAG, "Received data from ESP-NOW, forwarding to UART");

        // Send data to UART queue
        if (xQueueSend(uartQueue, &receivedData, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to send data to UART queue");
        }
    }

    static void onDataSend(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        ESP_LOGI(TAG, "Data sent via ESP-NOW to %02x:%02x:%02x:%02x:%02x:%02x, Status: %s",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5],
                 status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
    }

    void sendData(const Message &data)
    {
        if (esp_now_send(nullptr, reinterpret_cast<const uint8_t *>(&data), sizeof(data)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Error sending data via ESP-NOW");
        }
    }

    ~ESPNowHandler()
    {
        esp_now_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();
    }
};

// UART send task
void uartSendTask(void *pvParameters)
{
    Message data;

    while (true)
    {
        if (xQueueReceive(uartQueue, &data, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "Sending data over UART");
            std::string message = "MAC: " + std::to_string(data.macAddress[0]) + ":" +
                                  std::to_string(data.macAddress[1]) + " Value: " + std::to_string(data.value) + "\n";
            uart_write_bytes(UART_NUM, message.c_str(), message.length());
        }
    }
}

// ESP-NOW send task
void espNowSendTask(void *pvParameters)
{
    ESPNowHandler *handler = static_cast<ESPNowHandler *>(pvParameters);
    Message data;

    while (true)
    {
        if (xQueueReceive(espNowQueue, &data, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "Sending data over ESP-NOW");
            handler->sendData(data);
        }
    }
}

// UART receive task
void uartReceiveTask(void *pvParameters)
{
    uint8_t data[UART_BUF_SIZE];

    while (true)
    {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(1000));
        if (len > 0)
        {
            Message receivedMessage;
            // Parse UART data (assume MAC + Value)
            std::memcpy(receivedMessage.macAddress, data, 6);
            receivedMessage.value = *reinterpret_cast<float *>(data + 6);

            // Forward data to ESP-NOW
            if (xQueueSend(espNowQueue, &receivedMessage, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to send data to ESP-NOW queue");
            }
        }
    }
}

void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
}



// Initialize CDC-NCM
static void init_cdc_ncm()
{
    // Install TinyUSB driver
    tinyusb_config_t tusb_cfg = {};
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB driver installed");

    // Initialize Ethernet over USB
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_NCM();
    esp_netif_t *netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(netif));
    ESP_ERROR_CHECK(esp_netif_start(netif));
    ESP_LOGI(TAG, "CDC-NCM network interface initialized");
}


extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting ESP-NOW and UART Bridge");

    // Initialize queues
    espNowQueue = xQueueCreate(10, sizeof(Message));
    uartQueue = xQueueCreate(10, sizeof(Message));
    if (!espNowQueue || !uartQueue)
    {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }
    init_cdc_ncm();


    static ESPNowHandler espNowHandler;

    // Create tasks
    xTaskCreate(uartSendTask, "uartSendTask", 4096, &espNowHandler, 1, nullptr);
    xTaskCreate(espNowSendTask, "espNowSendTask", 4096, &espNowHandler, 1, nullptr);
    xTaskCreate(uartReceiveTask, "uartReceiveTask", 4096, nullptr, 1, nullptr);
}
