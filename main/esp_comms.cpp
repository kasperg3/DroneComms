#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include "nvs_flash.h"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include <iostream>
#include <stdio.h>
#include "driver/usb_serial_jtag.h"
#define MAC2STRING(mac) mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
#define CHECK_ERROR_AND_ABORT(ret, msg)                  \
    if (ret != ESP_OK)                                   \
    {                                                    \
        ESP_LOGE(TAG, msg ": %s", esp_err_to_name(ret)); \
        abort();                                         \
    }
#define BUF_SIZE (ESP_NOW_MAX_DATA_LEN_V2)
#define TAG "ESP-COMM"
using std::exception;
// Data structure for ESP-NOW

struct Message
{
    uint8_t *data;
    size_t len;
};

// Queue handle for ESP-NOW and UART
QueueHandle_t espNowTransmitQueue;
QueueHandle_t serialWriteQueue;

// Define ESPNowHandler class
class ESPNowHandler
{
private:
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
        auto wifi_ifx = WIFI_IF_STA;
        // Initialize Wi-Fi in station mode
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_protocol(wifi_ifx, WIFI_PROTOCOL_LR)); //
        ESP_ERROR_CHECK(esp_wifi_start());

        // Initialize ESP-NOW
        CHECK_ERROR_AND_ABORT(esp_now_init(), "ESP-NOW initialization failed")

        // Register callbacks
        esp_now_register_recv_cb(onDataReceive);
        esp_now_register_send_cb(onDataSend);

        // Add a broadcast peer
        esp_now_peer_info_t peerInfo{};
        std::memset(peerInfo.peer_addr, 0xFF, ESP_NOW_ETH_ALEN); // Broadcast address
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        peerInfo.ifidx = wifi_ifx;
        CHECK_ERROR_AND_ABORT(esp_now_add_peer(&peerInfo), "Failed to add broadcast peer");

        // Set rate config
        esp_now_rate_config_t rate_config = {
            .phymode = WIFI_PHY_MODE_LR,     // Set to WIFI_PHY_MODE_11B for short distance
            .rate = WIFI_PHY_RATE_LORA_500K, // Set to WIFI_PHY_RATE_LORA_500K for LoRa
            .ersu = true,                    // Enable ERSU (Extended Range Single User)
            .dcm = true                      //
        };

        CHECK_ERROR_AND_ABORT(esp_now_set_peer_rate_config(peerInfo.peer_addr, &rate_config), "Failed to set peer rate config");
        CHECK_ERROR_AND_ABORT(esp_wifi_set_max_tx_power(82), "Failed to set the tx power"); // Set maximum TX power (0-82, where 82 is the highest)
    }

    static void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
    {
        ESP_LOGI(TAG, "Data received from %02x:%02x:%02x:%02x:%02x:%02x, Length: %d",
                 MAC2STRING(info->src_addr), len);
        ESP_LOGI(TAG, "RSSI: %d dBm", info->rx_ctrl->rssi);

        Message message;
        message.data = (uint8_t *)malloc(len);
        if (message.data == nullptr)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for message data");
            return;
        }
        std::memcpy(message.data, data, len);
        message.len = len;
        ESP_LOGI(TAG, "Received data from ESP-NOW: %.*s Entries: %d", message.len, message.data, message.len);
        if (xQueueSend(serialWriteQueue, &message, portMAX_DELAY) != pdPASS)
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

    void sendData(const uint8_t *peer_addr, Message message)
    {
        if (message.len > ESP_NOW_MAX_DATA_LEN_V2)
        {
            ESP_LOGE(TAG, "Data size exceeds ESP-NOW maximum limit");
        }
        else
        {
            ESP_LOGI(TAG, "Sending data: %.*s, Length: %d", message.len, message.data, message.len);
            auto result = esp_now_send(peer_addr, message.data, message.len);
            if (result != ESP_OK)
            {
                ESP_LOGE(TAG, "Error sending data: %s", esp_err_to_name(result));
            }
        }
    }

    ~ESPNowHandler()
    {
        esp_now_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();
    }
};

class SerialHandler
{
private:
public:
    SerialHandler()
    {
        init();
    }

    int read(uint8_t *buffer, size_t buffer_size)
    {
        size_t total_len = 0;
        while (total_len < buffer_size)
        {
            int len = usb_serial_jtag_read_bytes(buffer + total_len, 1, pdMS_TO_TICKS(100));
            if (len > 0)
            {
                total_len += len;
                if (total_len >= 3 && buffer[total_len - 1] == '\x00' && buffer[total_len - 2] == '\x00' && buffer[total_len - 3] == '\x00')
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
        return total_len;
    }

    void write(uint8_t *serialMessage, size_t len)
    {
        usb_serial_jtag_write_bytes(serialMessage, len, 10 / portTICK_PERIOD_MS);
    }

    ~SerialHandler()
    {
    }

    void init()
    {
        usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .tx_buffer_size = BUF_SIZE * 4,
            .rx_buffer_size = BUF_SIZE * 4,
        };

        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    }
};

void espNowSendTask(void *pvParameters)
{
    uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast
    try
    {
        ESPNowHandler *handler = static_cast<ESPNowHandler *>(pvParameters);
        Message message;
        while (xQueueReceive(espNowTransmitQueue, &message, portMAX_DELAY) == pdPASS)
        {
            handler->sendData(broadcastAddress, message);
        }
    }
    catch (exception &e)
    {
        ESP_LOGE(TAG, "Exception: %s", e.what());
    }
}

void serialReadTask(void *pvParameters)
{
    try
    {
        SerialHandler *handler = static_cast<SerialHandler *>(pvParameters);

        uint8_t *serialMessage;
        serialMessage = (uint8_t *)malloc(BUF_SIZE);
        if (serialMessage == nullptr)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for serial message");
            abort();
        }
        while (1)
        {
            int len = handler->read(serialMessage, BUF_SIZE);

            if (len > 0)
            {
                Message message;
                message.data = (uint8_t *)malloc(len);
                if (message.data == nullptr)
                {
                    ESP_LOGE(TAG, "Failed to allocate memory for message data");
                    continue;
                }
                std::memcpy(message.data, serialMessage, len);
                message.len = len;
                ESP_LOGI(TAG, "Serial Data received: %.*s, Entries: %d", message.len, message.data, message.len);
                if (xQueueSend(espNowTransmitQueue, &message, portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGE(TAG, "Failed to send data to ESP-NOW queue");
                }
            }
        }
    }
    catch (exception &e)
    {
        ESP_LOGE(TAG, "Exception: %s", e.what());
    }
}

// Serial write task
void serialWriteTask(void *pvParameters)
{
    try
    {
        SerialHandler *handler = static_cast<SerialHandler *>(pvParameters);
        Message message;
        while (xQueueReceive(serialWriteQueue, &message, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "Writing data to serial: %.*s, Length: %d", message.len, message.data, message.len);
            handler->write(message.data, message.len);
        }
    }
    catch (exception &e)
    {
        ESP_LOGE("usb_serial_jtag echo", "Exception: %s", e.what());
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting ESP-NOW and UART Bridge");
    // Initialize queues

    espNowTransmitQueue = xQueueCreate(50, sizeof(Message));
    serialWriteQueue = xQueueCreate(50, sizeof(Message));
    if (!espNowTransmitQueue || !serialWriteQueue)
    {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

    static ESPNowHandler espNowHandler;
    static SerialHandler serialHandler;

    // Create tasks
    xTaskCreate(espNowSendTask, "espNowSendTask", 4096, &espNowHandler, 1, nullptr);
    xTaskCreate(serialReadTask, "serialTask", 4096, &serialHandler, 1, nullptr);
    xTaskCreate(serialWriteTask, "serialTask", 4096, &serialHandler, 1, nullptr);
}
