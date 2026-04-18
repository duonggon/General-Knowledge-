#include "over_the_air.h"

#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_log.h>
#include <esp_system.h>
#include <sys/param.h>
#include "driver/gpio.h"

#define _BLINK_GPIO GPIO_NUM_2 // LED on pin 2

static const char *P_TAG = "OTA_UPDATE";

// Web interface + JavaScript for OTA code upload via browser
static const char* P_INDEX_HTML = 
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Robot Delta OTA</title></head>"
    "<body style='text-align:center; font-family:Arial; margin-top:50px;'>"
    "<h2>UPDATE FIRMWARE (OTA)</h2>"
    "<input type='file' id='fileInput' accept='.bin' style='margin:20px;'><br>"
    "<button onclick='uploadFile()' style='padding:10px 20px; background:#4CAF50; color:white; border:none; border-radius:5px; cursor:pointer;'>Upload</button>"
    "<p id='status' style='font-weight:bold; margin-top:20px;'></p>"
    "<script>"
    "function uploadFile() {"
        "var fileInput = document.getElementById('fileInput');"
        "var status = document.getElementById('status');"
        "if (fileInput.files.length === 0) { status.innerHTML = 'No firmware.bin file selected'; status.style.color = 'red'; return; }"
        "var file = fileInput.files[0];"
        "status.innerHTML = '// Uploading code... Please do not close this page!'; status.style.color = 'blue';"
        "fetch('/update', { method: 'POST', body: file }).then(response => {"
            "if (response.ok) { status.innerHTML = 'UPLOAD SUCCESSFUL! The robot is restarting...'; status.style.color = 'green'; }"
            "else { status.innerHTML = 'Upload error!'; status.style.color = 'red'; }"
        "}).catch(err => { status.innerHTML = 'Network connection lost!'; status.style.color = 'red'; });"
    "}"
    "</script></body></html>";

// Handle access to the IP address (192.168.4.1)
static esp_err_t _Index_Get_Handler(httpd_req_t *req) {
    httpd_resp_send(req, P_INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handle when the "Update Firmware" button is pressed
static esp_err_t _Update_Post_Handler(httpd_req_t *req) {

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);

    ESP_LOGI(P_TAG, "Currently running at: %s (0x%08"PRIx32")", running->label, running->address);
    ESP_LOGI(P_TAG, "// Will be uploaded to: %s (0x%08"PRIx32")", update->label, update->address);
    
    char buf[1024];
    int received = 0;
    int remaining = req->content_len;
    bool led_state = false;

    // Find the empty App partition to overwrite
    const esp_partition_t *p_update_partition = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t update_handle = 0;

    if (p_update_partition == NULL) {
        ESP_LOGE(P_TAG, "No valid OTA partition found!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(P_TAG, "Start uploading code to partition: %s", p_update_partition->label);
    esp_err_t err = esp_ota_begin(p_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(P_TAG, "OTA initialization error (esp_ota_begin failed)");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Loop to receive each data chunk from the browser and write it to Flash
    while (remaining > 0) {
        if ((received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue; // If it is delayed a bit, keep waiting
            ESP_LOGE(P_TAG, "Error receiving data from the network!");
            esp_ota_end(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        // Write the received data chunk to Flash
        err = esp_ota_write(update_handle, (const void *)buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(P_TAG, "Flash write error (esp_ota_write failed)");
            esp_ota_end(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        remaining -= received;

        // ================= LIGHT EFFECT =================
        // Toggle the LED state every time a data chunk is written successfully
        led_state = !led_state;
        gpio_set_level(_BLINK_GPIO, led_state);
        // =====================================================
    }

    // Finish the upload session
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(P_TAG, "OTA termination error (esp_ota_end failed)");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Switch the boot partition to the newly uploaded App partition
    err = esp_ota_set_boot_partition(p_update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(P_TAG, "Error setting the boot partition (esp_ota_set_boot_partition failed)");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(P_TAG, "Code uploaded successfully! The robot will restart automatically...");
    httpd_resp_sendstr(req, "OK");
    
    // Indicate successful upload by turning the LED on for 2 seconds
    gpio_set_level(_BLINK_GPIO, 1);

    // Sleep for 2 seconds so the web response can finish, then reset the chip
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}

// Intercept handler (Captive Portal Handler)
static esp_err_t _Captive_Portal_Handler(httpd_req_t *req, httpd_err_code_t err) {
    // Redirect 100% of stray users back to the Robot home page
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

void OTA_Init_Web_Server() {
    // Configure the LED pin as Output
    gpio_reset_pin(_BLINK_GPIO);
    gpio_set_direction(_BLINK_GPIO, GPIO_MODE_OUTPUT); // configure the LED pin as Output
    gpio_set_level(_BLINK_GPIO, 0); // Turn off the LED during normal operation

    // Configure the Web Server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80; // Default Web port

    config.stack_size = 8192;        
    config.recv_wait_timeout = 10;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register the GET "/" home page route
        httpd_uri_t uri_get = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = _Index_Get_Handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_get);

        // Register the POST "/update" upload handling route
        httpd_uri_t uri_post = {
            .uri      = "/update",
            .method   = HTTP_POST,
            .handler  = _Update_Post_Handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_post);
        
        ESP_LOGI(P_TAG, "OTA Web Server is running! Visit http://192.168.4.1 to upload code.");
        // Register interception for all invalid URLs (404 Error)
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, _Captive_Portal_Handler);
    } else {
        ESP_LOGE(P_TAG, "Error: unable to start the Web Server!");
    }
}