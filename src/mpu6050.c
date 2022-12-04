#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include"parameters.h"

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
#define PI   3.14
float accAngleX,accAngleY,gyroAngleX, gyroAngleY, gyroAngleZ=0;

int16_t u_accel_x,u_accel_y,u_accel_z;
int16_t u_gyro_x,u_gyro_y,u_gyro_z;


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

	cmd2 = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd2));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd2, 0x1C, 0x10);
	i2c_master_write_byte(cmd2, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd2));
	i2c_master_cmd_begin(I2C_NUM_0, cmd2, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd2);




	uint8_t data[14];
    float elapsedTime=0.01;

	while(1) {

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

			
			u_accel_x = ((data[0] << 8) | data[1]);
			u_accel_y = ((data[2] << 8) | data[3]);
			u_accel_z = ((data[4] << 8) | data[5]);
			
			accel_x = u_accel_x / 1650;
			accel_y = u_accel_y / 1650;
			accel_z = u_accel_z / 1650;

			short temp = (data[6] << 8) | data[7];

			u_gyro_x = ((data[8] << 8) | data[9]);
			u_gyro_y = ((data[10] << 8) | data[11]);
			u_gyro_z = ((data[12] << 8) | data[13]);		
			
			
			gyro_x = u_gyro_x / 131.0;
			gyro_y = u_gyro_y / 131.0;
			gyro_z = u_gyro_z / 131.0;


			accAngleX = (atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
			accAngleY = (atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

		  	// Correct the outputs with the calculated error values
			gyro_x = gyro_x + 0.56; // GyroErrorX ~(-0.56)
			gyro_y = gyro_y - 2; // GyroErrorY ~(2)
			gyro_z = gyro_z + 0.79; // GyroErrorZ ~ (-0.8)

			// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
			gyroAngleX = gyroAngleX + gyro_x * elapsedTime; // deg/s * s = deg
			gyroAngleY = gyroAngleY + gyro_y * elapsedTime;
			yaw =  yaw + gyro_z * elapsedTime;

 			// Complementary filter - combine acceleromter and gyro angle values
			roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
			pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

			//ESP_LOGD(tag, "accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);
			//printf("accel_x: %f, accel_y: %f, accel_z: %f \n", accel_x, accel_y, accel_z);
			//printf("gyro_x: %f, gyro_y: %f, gyro_z: %f \n", gyro_x, gyro_y, gyro_z);
			//printf("roll: %0.1f, pitch: %0.1f, yaw: %0.1f \n", roll, pitch, yaw);
			
			temp_mpu=(temp/340)+36.53;
			//ESP_LOGD(tag, "accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);
			//printf("temp_mpu: %0.1f \n", temp_mpu);
			vTaskDelay(10/portTICK_PERIOD_MS);
		
		
	}

	vTaskDelete(NULL);
} // task_hmc5883l