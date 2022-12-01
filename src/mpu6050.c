#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "sdkconfig.h"

#include "globals.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_TEMP_OUT_H  0x41


#define MPU6050_PWR_MGMT_1   0x6B

/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x50 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */

static char tag[] = "mpu6050";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);



void mpu6050_task(void *ignore) {
	

	ESP_LOGD(tag, ">> mpu6050");
	i2c_config_t conf2;
	conf2.mode = I2C_MODE_MASTER;
	conf2.sda_io_num = PIN_SDA;
	conf2.scl_io_num = PIN_CLK;
	conf2.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf2.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf2.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf2));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    
	i2c_cmd_handle_t cmd2;
	vTaskDelay(200/portTICK_PERIOD_MS);

	cmd2 = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd2));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd2, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd2));
	i2c_master_cmd_begin(I2C_NUM_0, cmd2, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd2);

	cmd2 = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd2));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd2, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd2, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd2));
	i2c_master_cmd_begin(I2C_NUM_0, cmd2, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd2);


	uint8_t data[14];
	uint8_t gyro_data[14];



	while(1) {

	//	if(xSemaphoreTake(mutexHdlr, pdMS_TO_TICKS(500)))
	//	{

			// Tell the MPU6050 to position the internal register pointer to register
			// MPU6050_ACCEL_XOUT_H.
			cmd2 = i2c_cmd_link_create();
			ESP_ERROR_CHECK(i2c_master_start(cmd2));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, MPU6050_ACCEL_XOUT_H, 1));
			ESP_ERROR_CHECK(i2c_master_stop(cmd2));
			ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd2, 2000/portTICK_PERIOD_MS));
			i2c_cmd_link_delete(cmd2);

			cmd2 = i2c_cmd_link_create();
			ESP_ERROR_CHECK(i2c_master_start(cmd2));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data,   0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+1, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+2, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+3, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+4, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+5, 0));
	
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+6, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+7, 0));
			
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+8, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+9, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+10, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+11, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+12, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, data+13, 1));


			//i2c_master_read(cmd, data, sizeof(data), 1);
			ESP_ERROR_CHECK(i2c_master_stop(cmd2));
			ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd2, 2000/portTICK_PERIOD_MS));
			i2c_cmd_link_delete(cmd2);

			accel_x = (data[0] << 8) | data[1];
			accel_y = (data[2] << 8) | data[3];
			accel_z = (data[4] << 8) | data[5];

			short temp = (data[6] << 8) | data[7];

			gyro_x = (data[8] << 8) | data[9];
			gyro_y = (data[10] << 8) | data[11];
			gyro_z = (data[12] << 8) | data[13];


			//ESP_LOGD(tag, "accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);
			printf("accel_x: %d, accel_y: %d, accel_z: %d \n", accel_x, accel_y, accel_z);
			printf("gyro_x: %d, gyro_y: %d, gyro_z: %d \n", gyro_x, gyro_y, gyro_z);

			temp_mpu=(temp/340)+36.53;
			//ESP_LOGD(tag, "accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);
			printf("temp_mpu: %0.1f \n", temp_mpu);

			vTaskDelay(3000/portTICK_PERIOD_MS);
		
		
		//}
	}

	vTaskDelete(NULL);
} // task_hmc5883l