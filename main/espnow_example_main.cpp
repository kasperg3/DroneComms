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
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"


#define TAG "ESP-NOW"
#define UDP_PORT 12345 // Port for UDP communication
#define RECV_TASK_STACK_SIZE 4096
#define RECV_TASK_PRIORITY 5

static int udp_socket = -1;

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

void usb_init(void)
{
    // USB Device configuration
    const usb_device_config_t dev_config = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    // Install USB Device driver
    ESP_ERROR_CHECK(usb_device_install(&dev_config));

    // CDC-ACM configuration
    const cdc_acm_config_t cdc_acm_config = {
        .data_bit = CDC_DATA_8_BIT,
        .parity = CDC_PARITY_NONE,
        .stop_bit = CDC_STOP_BITS_1,
        .flow_ctrl = CDC_FLOW_CTRL_NONE,
        .rx_unread_buf_sz = 64,
        .tx_unread_buf_sz = 64,
    };

    // Install CDC-ACM driver
    ESP_ERROR_CHECK(cdc_acm_driver_install(&cdc_acm_config));
}

void usb_to_espnow_task(void *pvParameters)
{
    ESPNowHandler *espnow_handler = static_cast<ESPNowHandler *>(pvParameters);
    uint8_t data[64];
    size_t len;

    while (true)
    {
        // Read data from USB
        len = cdc_acm_read_bytes(data, sizeof(data), portMAX_DELAY);
        if (len > 0)
        {
            // Prepare ESP-NOW message
            Message msg;
            std::memcpy(msg.macAddress, broadcastAddr, 6);
            msg.value = parse_value_from_data(data, len); // Implement this function as needed

            // Send data via ESP-NOW
            espnow_handler->sendData(msg);
        }
    }
}

void espnow_to_usb_task(void *pvParameters)
{
    Message msg;

    while (xQueueReceive(dataQueue, &msg, portMAX_DELAY) == pdPASS)
    {
        // Format data as needed
        uint8_t data[64];
        size_t len = format_data_from_message(msg, data, sizeof(data)); // Implement this function

        // Send data over USB
        cdc_acm_write_bytes(data, len);
    }
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
    static ESPNowHandler espNowHandler;

    usb_init();

    xTaskCreate(usb_to_espnow_task, "USBToESPNowTask", 4096, &espNowHandler, 1, nullptr);
    xTaskCreate(espnow_to_usb_task, "ESPNowToUSBTask", 4096, nullptr, 2, nullptr);

}
