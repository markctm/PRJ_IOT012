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
#include <freertos/semphr.h>

// SDA - GPIO21
#define PIN_SDA 33
// SCL - GPIO22
#define PIN_SCL 32
static const char* TAG = "ssd1306";


#define BATTERY_10           6


#define BitSet(variavel,bit) (variavel |= (1<<bit))
#define BitClr(variavel,bit) (variavel &= ~(1<<bit))
#define BitTst(variavel,bit) (variavel & (1<<bit))


xTaskHandle notified_hdlr = NULL;
u8g2_t u8g2; 
u8g2_t u8g2_status_bar; 
SemaphoreHandle_t mutexHdlr;

uint32_t Status_bar_flag = 0b00000000000000000000000000000000;    //  0b0000000|0000|00000; 




static void task_test_SSD1306i2c(u8g2_t* u8g2_display);
static void prvTask_StatusBar(u8g2_t* pvParameters);
void SIGNAL_STRENGHT_5();
void SIGNAL_STRENGHT_4();
void Battery_Level();
/*
const uint8_t ROW[5]={0,15,31,47,63};

void OledOverlayPrint(char * text, uint8_t row)
{

 u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
 u8g2_SetDrawColor(0);
 u8g2_DrawBox(pvParameters,0,ROW[row-1]) u8g2_GetDisplayWidth()-1,ROW[row +1] - ROW[row+1])

}

*/

void Battery_Level()
{
  BitSet(Status_bar_flag,5);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 

}

void SERVER_CONNECTED()
{
  BitSet(Status_bar_flag,9);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}
void SERVER_DISCONNECTED()
{
  BitClr(Status_bar_flag,9);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}




void SIGNAL_STRENGHT_5()
{
  BitSet(Status_bar_flag,4);
  BitClr(Status_bar_flag,3);
  BitClr(Status_bar_flag,2);
  BitClr(Status_bar_flag,1);
  BitClr(Status_bar_flag,0);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}

void SIGNAL_STRENGHT_4()
{
  BitClr(Status_bar_flag,4);
  BitSet(Status_bar_flag,3);
  BitClr(Status_bar_flag,2);
  BitClr(Status_bar_flag,1);
  BitClr(Status_bar_flag,0);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}

void SIGNAL_STRENGHT_3()
{
  BitClr(Status_bar_flag,4);
  BitClr(Status_bar_flag,3);
  BitSet(Status_bar_flag,2);
  BitClr(Status_bar_flag,1);
  BitClr(Status_bar_flag,0);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}

void SIGNAL_STRENGHT_2()
{
  BitClr(Status_bar_flag,4);
  BitClr(Status_bar_flag,3);
  BitClr(Status_bar_flag,2);
  BitSet(Status_bar_flag,1);
  BitClr(Status_bar_flag,0);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}
void SIGNAL_STRENGHT_1()
{
  BitClr(Status_bar_flag,4);
  BitClr(Status_bar_flag,3);
  BitClr(Status_bar_flag,2);
  BitClr(Status_bar_flag,1);
  BitSet(Status_bar_flag,0);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}
void NO_SIGNAL()
{
  BitClr(Status_bar_flag,4);
  BitClr(Status_bar_flag,3);
  BitClr(Status_bar_flag,2);
  BitClr(Status_bar_flag,1);
  BitClr(Status_bar_flag,0);
  xTaskNotify(notified_hdlr, Status_bar_flag, eSetValueWithOverwrite); 
}



void vInit_display()
{
  
 
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda = PIN_SDA;
  u8g2_esp32_hal.scl = PIN_SCL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

   // a structure which will contain all the data for one display
  u8g2_Setup_ssd1306_i2c_128x32_univision_f(
      &u8g2, U8G2_R0,
      // u8x8_byte_sw_i2c,
      u8g2_esp32_i2c_byte_cb,
      u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2);  // send init sequence to the display, display is in
                            // sleep mode after this,

  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0);  // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);



    mutexHdlr = xSemaphoreCreateMutex();
    if (mutexHdlr == NULL)
    {
        printf("Impossible situation\n");
        return;
    }



   xTaskCreate(task_test_SSD1306i2c, "LCD_task2", configMINIMAL_STACK_SIZE * 3, &u8g2, 2, NULL);
   xTaskCreate(prvTask_StatusBar, "Notified", configMINIMAL_STACK_SIZE*3, &u8g2, 2, &notified_hdlr);

  Battery_Level();

}



static void prvTask_StatusBar(u8g2_t* pvParameters)
{
    uint32_t notificationValue;

    for (;;)
    {
        if (xTaskNotifyWait(ULONG_MAX,ULONG_MAX,&notificationValue,portMAX_DELAY))
        {
           
           
           if(xSemaphoreTake(mutexHdlr, pdMS_TO_TICKS(500)))
           {
                  
            
            //--------- Connection With Server ------------------------
    
            if (BitTst(notificationValue,9))
            {
              
              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,82,0,35,8);         
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_m2icon_9_tf);            
              u8g2_DrawGlyph(pvParameters,80,8,68);          
           }
           else
           {

              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,82,0,35,8);     
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_m2icon_9_tf);            
              u8g2_DrawGlyph(pvParameters,80,8,67);

           }
                     
            //--------- Battery ------------------------
    
            if (BitTst(notificationValue,5))
            {
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
              u8g2_DrawGlyph(pvParameters,0,8,57487);
              
              u8g2_SetFont(pvParameters, u8g2_font_tinytim_tf );
              u8g2_DrawStr(pvParameters, 11, 8, "10%");
            
            }
            
            //--------- RADIO ------------------------
            
            if (BitTst(notificationValue,4))
            { 
              
              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,115,0,27,6);
              
              // Signal Strenght 5
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
              u8g2_DrawGlyph(pvParameters,115,6,57948);
            }
            if (BitTst(notificationValue,3))
            {   
              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,115,0,27,6);
              
              // Signal Strenght 5
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
              u8g2_DrawGlyph(pvParameters,115,6,57947);
            }
            if (BitTst(notificationValue,2))
            {   
              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,115,0,27,6);
              
              // Signal Strenght 5
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
              u8g2_DrawGlyph(pvParameters,115,6,57946);
            }
            if (BitTst(notificationValue,1))
            {   
              
              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,115,0,27,6);
              
              // Signal Strenght 5
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
              u8g2_DrawGlyph(pvParameters,115,6,57945);
            }
            if ((BitTst(notificationValue,4)==0) && (BitTst(notificationValue,3)==0) && (BitTst(notificationValue,2)==0) && (BitTst(notificationValue,1)==0) &&(BitTst(notificationValue,0)==0))
            {
              
              u8g2_SetDrawColor(pvParameters,0);
              u8g2_DrawBox(pvParameters,115,0,27,6);
              
              u8g2_SetDrawColor(pvParameters,1);
              u8g2_SetFont(pvParameters,u8g2_font_siji_t_6x10);
              u8g2_DrawGlyph(pvParameters,115,6,57944);
              u8g2_SetFont(pvParameters,u8g2_font_4x6_tf);
              u8g2_DrawGlyph(pvParameters,119,7,215);

            }
            u8g2_SendBuffer(pvParameters);
          }
              
         xSemaphoreGive(mutexHdlr);
        }

    }
    vTaskDelete(NULL);
}




static void task_test_SSD1306i2c(u8g2_t* u8g2_display) {

char text[20];

    for(;;)
    {
        if(xSemaphoreTake(mutexHdlr, pdMS_TO_TICKS(500)))
        {   
            
          u8g2_SetDrawColor(u8g2_display,0);
          u8g2_DrawBox(u8g2_display,0,14,127,63);
          u8g2_SetDrawColor(u8g2_display,1);

          u8g2_SetFont(u8g2_display, u8g2_font_micro_tr );
/*
          u8g2_DrawStr(u8g2_display, 3, 14, "Humidity:");
          sprintf(text, "%0.1f", humidity);
          u8g2_DrawStr(u8g2_display, 41,14, text);

          u8g2_DrawStr(u8g2_display, 3, 21, "Temperature:");
          sprintf(text, "%0.1f", temperature);
          u8g2_DrawStr(u8g2_display, 52,21, text);
          u8g2_SendBuffer(u8g2_display);
*/


          u8g2_DrawStr(u8g2_display, 3, 14, "acel(x,y,z): ");
          
          int pos=0;

          /*u8g2_SetDrawColor(u8g2_display,0);
          u8g2_DrawBox(u8g2_display,0,14,74,7);
          u8g2_SetDrawColor(u8g2_display,1);*/

          pos+=sprintf(&text[pos], "%0.0f   ", accel_x);
          pos+=sprintf(&text[pos], "%0.0f   ", accel_y);
          pos+=sprintf(&text[pos], "%0.0f   ", accel_z);
          u8g2_DrawStr(u8g2_display,54,14, text);
          
          
          u8g2_DrawStr(u8g2_display, 3, 21, "gyro(x,y,z): ");
          
          pos=0;
          pos+=sprintf(&text[pos], "%0.1f ", gyro_x);
          pos+=sprintf(&text[pos], "%0.1f ", gyro_y);
          pos+=sprintf(&text[pos], "%0.1f ", gyro_z);

          u8g2_DrawStr(u8g2_display, 55,21, text);


          u8g2_DrawStr(u8g2_display, 3, 28, "Lumin: ");
          pos=0;
          pos+=sprintf(&text[pos], "%0.1f ", illuminance);
          u8g2_DrawStr(u8g2_display, 35,28, text);

          u8g2_DrawStr(u8g2_display, 70, 28, "Temp: ");
          pos=0;
          pos+=sprintf(&text[pos], "%0.1f ", temperature);
          u8g2_DrawStr(u8g2_display, 95,28, text);


/*
          u8g2_DrawStr(u8g2_display, 3, 21, "accel_y: ");
          sprintf(text, "%d", accel_y);
          u8g2_DrawStr(u8g2_display, 41,21, text);

          u8g2_DrawStr(u8g2_display, 3, 28, "accel_z: ");
          sprintf(text, "%d", accel_y);
          u8g2_DrawStr(u8g2_display, 41,28, text);
*/


          u8g2_SendBuffer(u8g2_display);




        //  ESP_LOGI(TAG, "All done!");
        }

        xSemaphoreGive(mutexHdlr);


        vTaskDelay(pdMS_TO_TICKS(20));

    }
  vTaskDelete(NULL);

}