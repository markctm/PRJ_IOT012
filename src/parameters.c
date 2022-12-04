

#include <stdio.h>
#include "cJSON.h"
#include <string.h>



float limiar_accel_x,limiar_accel_y,limiar_accel_z;
float limiar_gyro_x,limiar_gyro_y,limiar_gyro_z;
float limiar_roll, limiar_pitch, limiar_yaw;
float limiar_temp_mpu;
float limiar_illuminance;


void receivemsg_task(int len, char *msg)
{
    
  printf("%s", msg);

  cJSON *method;
  cJSON *params;
  cJSON *root = cJSON_Parse(msg);
  method = cJSON_GetObjectItem(root, "method");
  params = cJSON_GetObjectItem(root, "params");
  
  printf("%s\n", method->valuestring);
  printf("%s\n", params->valuestring);


    if(strcmp(method->valuestring,"setValue_acel_x")==0)
    {
        
        limiar_accel_x=atoi(params->valuestring);
       
    }

    if(strcmp(method->valuestring,"setValue_acel_y")==0)
    {
        limiar_accel_y=atoi(params->valuestring);
    }


    if(strcmp(method->valuestring,"setValue_acel_z")==0)
    {
        limiar_accel_z=atoi(params->valuestring);
    }


    if(strcmp(method->valuestring,"setValue_gyro_x")==0)
    {
       limiar_gyro_x=atoi(params->valuestring);
    }


    if(strcmp(method->valuestring,"setValue_gyro_y")==0)
    {
        limiar_gyro_y=atoi(params->valuestring);
    }


    if(strcmp(method->valuestring,"setValue_gyro_z")==0)
    {
         limiar_gyro_z=atoi(params->valuestring);
    }


    if(strcmp(method->valuestring,"setValue_lumin")==0)
    {
        limiar_illuminance=atoi(params->valuestring);
    }


    if(strcmp(method->valuestring,"setValue_temp")==0)
    {
        limiar_temp_mpu=atoi(params->valuestring);
    }

}
