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
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"

#define BUF_SIZE (1024)
#define ECHO_TASK_STACK_SIZE (4096)

#define TAG "ESP-NOW"
using std::exception;
// Data structure for ESP-NOW

struct Message
{
    uint8_t data[BUF_SIZE];
    size_t len;
};

// Queue handle for ESP-NOW and UART
QueueHandle_t transmitQueue;
QueueHandle_t recieveQueue;

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
        peerInfo.ifidx = WIFI_IF_STA;

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

        // Send data to serial queue
        if (xQueueSend(recieveQueue, &receivedData, portMAX_DELAY) != pdPASS)
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

    void sendData(const uint8_t *peer_addr, const Message &data)
    {

        auto result = esp_now_send(peer_addr, data.data, data.len);
        if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "Error sending data: %s", esp_err_to_name(result));
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

    Message read()
    {
        Message message;
        message.len = usb_serial_jtag_read_bytes(message.data, sizeof(message.data), 20 / portTICK_PERIOD_MS);
        return message;
    }

    void write(Message serialMessage)
    {
        usb_serial_jtag_write_bytes(serialMessage.data, serialMessage.len, 20 / portTICK_PERIOD_MS);
    }

    ~SerialHandler()
    {
    }

    void init()
    {
        // Configure USB SERIAL JTAG
        usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .tx_buffer_size = BUF_SIZE,
            .rx_buffer_size = BUF_SIZE,
        };

        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
        ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");
    }
};

// ESP-NOW send task
void espNowSendTask(void *pvParameters)
{
    try
    {
        ESPNowHandler *handler = static_cast<ESPNowHandler *>(pvParameters);
        Message message;

        while (xQueueReceive(transmitQueue, &message, portMAX_DELAY) == pdPASS)
        {
            uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast
            handler->sendData(broadcastAddress, message);
        }
    }
    catch (exception &e)
    {
        ESP_LOGE(TAG, "Exception: %s", e.what());
    }
}
// Serial read task
void serialReadTask(void *pvParameters)
{
    try
    {
        SerialHandler *handler = static_cast<SerialHandler *>(pvParameters);
        while (1)
        {
            Message serialMessage = handler->read();
            if (serialMessage.len > 0)
            {
                ESP_LOGI(TAG, "Serial Data received: %.*s, Length: %d", serialMessage.len, serialMessage.data, serialMessage.len);

                if (xQueueSend(transmitQueue, &serialMessage, portMAX_DELAY) != pdPASS)
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
        Message receivedData;

        while (xQueueReceive(recieveQueue, &receivedData, portMAX_DELAY) == pdPASS)
        {
            Message serialMessage;
            std::memcpy(serialMessage.data, &receivedData, sizeof(receivedData));
            serialMessage.len = sizeof(receivedData);

            handler->write(serialMessage);
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

    transmitQueue = xQueueCreate(10, sizeof(Message));
    recieveQueue = xQueueCreate(10, sizeof(Message));
    if (!transmitQueue || !recieveQueue)
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
