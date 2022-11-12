#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"

#include <u8g2.h>
#include "u8g2_esp32_hal.h"
#include <dht.h>
#include "vDisplay.h"
#include "globals.h"
#include "wifi.h"
#include "nvs_flash.h"
#include <string.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_connection.h"
#include "cJSON.h"
#include "esp_http_client.h"


static const char *TAG = "JSON";


#define SENSOR_TYPE 0 
#define CONFIG_EXAMPLE_DATA_GPIO 17
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define LED_RED (5)
#define LED_GREEN (18)
#define LED_BLUE (19)

#define RGB_OFF 0b000
#define RGB_RED 0b001
#define RGB_GREEN 0b010
#define RGB_YELLOW 0b011
#define RGB_BLUE 0b100
#define RGB_MAGENTA 0b101
#define RGB_CYAN 0b110
#define RGB_WHITE 0b111


float temperature, humidity;


esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt);

void vInitHW(void);

void vInitHW(void)
{
    gpio_pad_select_gpio(LED_RED);
    gpio_set_direction(LED_RED,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_GREEN);
    gpio_set_direction(LED_GREEN,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_BLUE);
    gpio_set_direction(LED_BLUE,GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void set_rgb_color(uint8_t color)
{
  
    gpio_set_level(LED_RED, bitRead(color, 0));
    gpio_set_level(LED_GREEN, bitRead(color, 1));
    gpio_set_level(LED_BLUE, bitRead(color, 2));
}

void dht_test(void *pvParameters)
{
   char msg[30];

    while (1)
    {
        if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
        {    printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
             //sprintf(msg, "temp: %0.1f", temperature);
          
           
            ESP_LOGI(TAG, "Serialize.....");
            cJSON *root;
            root = cJSON_CreateObject();
            esp_chip_info_t chip_info;
            esp_chip_info(&chip_info);       
            cJSON_AddNumberToObject(root, "temperature", temperature);
            cJSON_AddNumberToObject(root, "humidity", humidity);
            char *my_json_string = cJSON_Print(root);
            ESP_LOGI(TAG, "my_json_string\n%s",my_json_string);
            cJSON_Delete(root);

            
            esp_http_client_config_t config_post = {
                .url = "https://thingsboard.cloud/api/v1/6p3BVNpens9uZlpfG0Ah/telemetry",
                //.url = "https://ptsv2.com/t/y583v-1664319824/post",
                .method = HTTP_METHOD_POST,
                .cert_pem = NULL,
                .event_handler = client_event_post_handler};
                
            esp_http_client_handle_t client = esp_http_client_init(&config_post);

            esp_http_client_set_post_field(client, my_json_string, strlen(my_json_string));
            esp_http_client_set_header(client, "Content-Type", "application/json");

            esp_http_client_perform(client);
            esp_http_client_cleanup(client);            
            
           

            
            mqtt_publish(my_json_string);     
        }
        else
        {
            printf("Could not read data from sensor\n");
        
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void led_task(void *pvParameters)
{
 
    while (1)
    {
        set_rgb_color(RGB_RED);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_rgb_color(RGB_YELLOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_rgb_color(RGB_BLUE);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_rgb_color(RGB_WHITE);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_rgb_color(RGB_GREEN);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_rgb_color(RGB_MAGENTA);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}






void app_main()
{
    vInitHW();
    vInit_display();
    wifi_init_sta();

    //vTaskDelay(pdMS_TO_TICKS(3000));
    
    xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
    xTaskCreate(led_task, "led_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);

}

