#include "wifi_app.h"

// ========================= ESP-IDF system libraries =========================
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "lwip/sockets.h"
//#include <string.h>

static const char *_TAG = "WIFI_AP_LIB";

// WiFi event handler function (when a device connects or disconnects)
static void _Wifi_Event_Handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(_TAG, "Đã có thiết bị kết nối! MAC: " MACSTR, MAC2STR(event->mac));
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(_TAG, "A device has just disconnected! MAC: " MACSTR, MAC2STR(event->mac));
    }
}

// Function to initialize the ESP32 as a Wi-Fi Access Point (WiFi name, password, maximum allowed connections)
void Wifi_Init(const char* WIFI_AP_SSID, const char* WIFI_AP_PASS, uint8_t MAX_STA_CONN) {
    // Initialize NVS (Non-Volatile Storage) to store WiFi configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(_TAG, "Starting the Wi-Fi Access Point...");

    // Initialize the TCP/IP stack and Event Loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // Initialize WiFi with the default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register the event handler function (_Wifi_Event_Handler)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &_Wifi_Event_Handler, NULL, NULL));

    // Configure Access Point parameters (SSID, password, max connection)
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, WIFI_AP_SSID);
    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    strcpy((char*)wifi_config.ap.password, WIFI_AP_PASS);
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (strlen(WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // Enable AP mode and start broadcasting WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Prevent Wi-Fi from sleeping, force it to run at full power 100% of the time!
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(_TAG, "=========================================");
    ESP_LOGI(_TAG, "The robot Wi-Fi is ready!!");
    ESP_LOGI(_TAG, "Default IP address of the robot: 192.168.4.1");
    ESP_LOGI(_TAG, "=========================================");
}


// DNS Server task to trick devices
void Wifi_DNS_Server_Task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(53); // 53 is the standard DNS port
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    while (true) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        
        // Wait for the device to ask for directions
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 16, 0, (struct sockaddr *)&source_addr, &socklen);
        
        if (len > 12) {
            // Turn the device's question into an answer (DNS Spoofing)
            rx_buffer[2] |= 0x80; // Change the flag to Response
            rx_buffer[3] |= 0x80; // Allow recursion
            rx_buffer[6] = 0; rx_buffer[7] = 1; // Number of answers = 1

            // Inject the ESP32 IP (192.168.4.1) into the end of the packet
            rx_buffer[len++] = 0xC0; rx_buffer[len++] = 0x0C; // Pointer
            rx_buffer[len++] = 0x00; rx_buffer[len++] = 0x01; // Type A
            rx_buffer[len++] = 0x00; rx_buffer[len++] = 0x01; // Class IN
            rx_buffer[len++] = 0x00; rx_buffer[len++] = 0x00; // TTL
            rx_buffer[len++] = 0x00; rx_buffer[len++] = 0x3C; // TTL 60s
            rx_buffer[len++] = 0x00; rx_buffer[len++] = 0x04; // Data Length (4 bytes)
            
            // IP 192.168.4.1
            rx_buffer[len++] = 192;  rx_buffer[len++] = 168; 
            rx_buffer[len++] = 4;    rx_buffer[len++] = 1;   

            // Send it back to the phone
            sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
        }
    }
}