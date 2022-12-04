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
#include "mpu6050.h"
#include "driver/adc.h"


static const char *TAG = "JSON";


#define SENSOR_TYPE 0 
#define CONFIG_EXAMPLE_DATA_GPIO 17
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define LED_RED (5)
#define LED_GREEN (18)
#define LED_BLUE (19)
#define ADC1_0_PIN 36
#define BUZZER (4)


#define RGB_OFF 0b000
#define RGB_RED 0b001
#define RGB_GREEN 0b010
#define RGB_YELLOW 0b011
#define RGB_BLUE 0b100
#define RGB_MAGENTA 0b101
#define RGB_CYAN 0b110
#define RGB_WHITE 0b111



//------ Variaveis Globais---------

float temperature, humidity;
float accel_x,accel_y,accel_z;
float gyro_x,gyro_y,gyro_z;
float roll, pitch, yaw;
float temp_mpu;
float illuminance;

float limiar_accel_x,limiar_accel_y,limiar_accel_z;
float limiar_gyro_x,limiar_gyro_y,limiar_gyro_z;
float limiar_roll, limiar_pitch, limiar_yaw;
float limiar_temp_mpu;
float limiar_illuminance;
uint8_t flag_alarme=0;
unsigned char mac_base[6] = {0};


void vInitHW(void);


void vInitHW(void)
{
    gpio_pad_select_gpio(LED_RED);
    gpio_set_direction(LED_RED,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_GREEN);
    gpio_set_direction(LED_GREEN,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_BLUE);
    gpio_set_direction(LED_BLUE,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BUZZER);
    gpio_set_direction(BUZZER,GPIO_MODE_OUTPUT);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

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
        {           
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
        if(flag_alarme)
        {
            set_rgb_color(RGB_RED);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_rgb_color(RGB_YELLOW);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        else
        {
            set_rgb_color(RGB_GREEN);
             vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
    }
}

void temt6000_task()
{
   
    while(1)
    {   
        int vadc = adc1_get_raw(ADC1_CHANNEL_0);
        // ConversÃ£o de corrente no resistor de 10KOhms para iluminÃ¢ncia:
        // de acordo com o datasheet; 20 lux @ 10uA -> 2 lux/uA:
        // IluminÃ¢ncia[lux] = ((Vadc[mV] / 1000) / 10000) * 2000000

        illuminance = (((float)vadc / 1000) / 10000) * 2000000;
        //printf("\r\nIluminÃ¢ncia: %2.2f lux (Vadc=%d mV)\n",illuminance, vadc);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void sendmsg_task()
{
    
    while(1)
    {   
            cJSON *root;
            root = cJSON_CreateObject();
            esp_chip_info_t chip_info;
            esp_chip_info(&chip_info);       
            cJSON_AddNumberToObject(root, "accer_x", accel_x);
            cJSON_AddNumberToObject(root, "accer_y", accel_y);
            cJSON_AddNumberToObject(root, "accer_z", accel_z);
            cJSON_AddNumberToObject(root, "roll", gyro_x);
            cJSON_AddNumberToObject(root, "pitch",gyro_y);
            cJSON_AddNumberToObject(root,  "yaw", gyro_z);
            cJSON_AddNumberToObject(root, "luminancia", illuminance);
            cJSON_AddNumberToObject(root, "temperature", temperature);
            cJSON_AddNumberToObject(root, "humidity", humidity);

            char *my_json_string = cJSON_Print(root);
            ESP_LOGI(TAG, "my_json_string\n%s",my_json_string);
            
            cJSON_Delete(root);


          
            mqtt_publish(my_json_string,"tb/mqtt-integration-tutorial/sensors/Smart sensor/temperature",strlen(my_json_string));        
            vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void alarme_task()
{

    char  msg_alarme[80]="";
    char topic[50]="";
    sprintf(topic,"it012/warnings/%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0],mac_base[1],mac_base[2],mac_base[3],mac_base[4],mac_base[5]);
    printf("\n\n%s",topic);

    while(1)
    {   
         flag_alarme=0;
           printf("\n\n%s",topic);
        if(accel_x> limiar_accel_x)
        {
            flag_alarme=1;
            sprintf(msg_alarme, "Warning accel_x  %f > limiar_accel_x: %f", accel_x, limiar_accel_x);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  
          
        }
        if(accel_y> limiar_accel_y)
        {
            flag_alarme=1;
            sprintf(msg_alarme, "Warning accel_y  %f > limiar_accel_y: %f", accel_y, limiar_accel_y);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  


        }
        if(accel_z> limiar_accel_z)
        {
            flag_alarme=1;
            sprintf(msg_alarme, "Warning accel_z  %f > limiar_accel_z: %f", accel_z, limiar_accel_z);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  


        }
        if(gyro_x> limiar_gyro_x){
 
            flag_alarme=1;
            sprintf(msg_alarme, "Warning gyro_x  %f > limiar_gyro_x: %f", gyro_x, limiar_gyro_x);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  

        }   
        if(gyro_y> limiar_gyro_y) {
            flag_alarme=1;
            sprintf(msg_alarme, "Warning gyro_y  %f > limiar_gyro_y: %f", gyro_y, limiar_gyro_y);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  
 
 
        }
        if(gyro_z> limiar_gyro_z){
           
            flag_alarme=1;
            sprintf(msg_alarme, "Warning gyro_z  %f > limiar_gyro_z: %f", gyro_z, limiar_gyro_z);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));    
        }
        if(temp_mpu> limiar_temp_mpu){
           flag_alarme=1;

            sprintf(msg_alarme, "Warning temp_mpu  %f > limiar_temp_mpu: %f", temp_mpu, limiar_temp_mpu);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  
        }
        if(illuminance> limiar_illuminance){
            flag_alarme=1;

            sprintf(msg_alarme, "Warning illuminance  %f > limiar_illuminance: %f", illuminance, limiar_illuminance);
            printf("\n%s",msg_alarme);
            mqtt_publish(msg_alarme,topic,strlen(msg_alarme));  
        }

        if(flag_alarme)
        {
            printf("\nAlarme Disparado");
            gpio_set_level(BUZZER, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(BUZZER, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            printf("\nAlarme nao  Disparado");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}


void app_main()
{
    vInitHW();
    
    
    //Incializa display 
    vInit_display();
 

    // Task do Sensor DHT
    xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 5, NULL, 3, NULL);
    // Task do Sensor do Sensor MPU6050
    xTaskCreate(mpu6050_task, "mpu_task", configMINIMAL_STACK_SIZE * 5, NULL, 3, NULL);
    // Task do Sensor do Sensor Temt6000
    xTaskCreate(temt6000_task, "temt6000_task", configMINIMAL_STACK_SIZE * 3, NULL, 2, NULL);

    //Inicializa conexão com Wifi após inicializacao de sensores
    wifi_init_sta();
    
    //Formata dados em Json e envia para broker publico 
    xTaskCreate(sendmsg_task, "sendmsg_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);

    //Task do Led de indicacao 
    xTaskCreate(led_task, "led_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);

    //Inicializa task de monitoracao e disparo de alarme
    xTaskCreate(alarme_task, "alarme_task", configMINIMAL_STACK_SIZE * 3, NULL, 3, NULL);
}

