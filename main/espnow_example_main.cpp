#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <cstring>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_random.h"
#include "nvs_flash.h"
#define TAG "ESP-NOW"

// Data structure for ESP-NOW
struct Message
{
    uint8_t macAddress[6];
    float value;
};

// Peer information
const uint8_t broadcastAddr[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t peerAddress[ESP_NOW_ETH_ALEN] = {0x64, 0xE8, 0x33, 0x86, 0x81, 0xA4};

// 64:e8:33:86:81:a4
// 64:e8:33:84:b8:5c

// Queue handle
QueueHandle_t dataQueue;

auto WIFI_MODE = WIFI_MODE_STA;
auto WIFI_INTERFACE = WIFI_IF_STA;

static void wifi_init(void)
{

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // long range mode
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_INTERFACE, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
}

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
        wifi_init();

        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set Wi-Fi mode: %s", esp_err_to_name(ret));
            abort();
        }

        ret = esp_wifi_start();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
            abort();
        }

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

        // Add the peer
        esp_now_peer_info_t peerInfo{};
        std::memcpy(peerInfo.peer_addr, broadcastAddr, ESP_NOW_ETH_ALEN);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_INTERFACE;

        ret = esp_now_add_peer(&peerInfo);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
            abort();
        }
    }

    // Static callback for receiving data
    static void onDataReceive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
    {
        Message receivedData;
        std::memcpy(&receivedData, data, sizeof(receivedData));
        ESP_LOGI(TAG, "Received data from: %02x:%02x:%02x:%02x:%02x:%02x, Target MAC address: %02x:%02x:%02x:%02x:%02x:%02x, Value: %.2f",
                 esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2],
                 esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5],
                 receivedData.macAddress[0], receivedData.macAddress[1], receivedData.macAddress[2],
                 receivedData.macAddress[3], receivedData.macAddress[4], receivedData.macAddress[5],
                 receivedData.value);
        // Send received data to the queue
        if (xQueueSend(dataQueue, &receivedData, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to send data to queue");
        }
    }

    // Static callback for sending data
    static void onDataSend(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        ESP_LOGI(TAG, "Data sent to: %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGI(TAG, "Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
    }

    // Send data to the peer
    void sendData(const Message &data)
    {
        if (esp_now_send(broadcastAddr, reinterpret_cast<const uint8_t *>(&data), sizeof(data)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Error sending data");
        }
    }

    ~ESPNowHandler()
    {
        // Deinitialize ESP-NOW
        esp_now_deinit();

        // Stop Wi-Fi
        esp_wifi_stop();

        // Deinitialize Wi-Fi
        esp_wifi_deinit();
    }
};

void sendTask(void *pvParameters)
{
    ESPNowHandler *handler = static_cast<ESPNowHandler *>(pvParameters);
    Message data;

    while (true)
    {
        // Prepare data to send
        std::memcpy(data.macAddress, broadcastAddr, 6);
        data.value = esp_random() / static_cast<float>(UINT32_MAX) * 100.0f;

        // Send data
        handler->sendData(data);

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task for receiving data from the queue
void receiveTask(void *pvParameters)
{
    Message data;

    while (xQueueReceive(dataQueue, &data, portMAX_DELAY) == pdPASS)
    {
        ESP_LOGI(TAG, "Processing received data: MAC Address: %02x:%02x:%02x:%02x:%02x:%02x, Value: %.2f",
                 data.macAddress[0], data.macAddress[1], data.macAddress[2],
                 data.macAddress[3], data.macAddress[4], data.macAddress[5],
                 data.value);

        // Check if the MAC address is in the peer list
        esp_now_peer_info_t *peer = nullptr;
        if (!esp_now_is_peer_exist(data.macAddress))
        {
            ESP_LOGI(TAG, "Adding new peer");
            peer = (esp_now_peer_info_t *)malloc(sizeof(esp_now_peer_info_t));
            if (peer == nullptr)
            {
                ESP_LOGE(TAG, "Failed to allocate memory for new peer");
                continue;
            }
            std::memset(peer, 0, sizeof(esp_now_peer_info_t));
            std::memcpy(peer->peer_addr, data.macAddress, 6);
            peer->channel = 0;
            peer->encrypt = false;
            if (esp_now_add_peer(peer) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to add new peer");
                free(peer);
                continue;
            }
            free(peer);
        }
    }
}



// C entry point for the application
extern "C" void app_main()
{
    ESP_LOGI(TAG, "ESP-NOW Example Starting");
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Create the queue
    dataQueue = xQueueCreate(10, sizeof(Message));
    if (dataQueue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    static ESPNowHandler espNowHandler; // Define handler here

    // Start the main task
    xTaskCreate(sendTask, "SendTask", 4096, &espNowHandler, 1, nullptr);
    xTaskCreate(receiveTask, "ReceiveTask", 4096, &espNowHandler, 2, nullptr);
}
