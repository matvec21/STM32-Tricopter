#pragma once

typedef struct
{
//	float pitch;
//	float yaw;
//	float roll; no use
	float vx;
	float vy;
} MPU6050_Data;

extern MPU6050_Data mpu6050;

void MPU6050_Init();

void MPU6050_Calibrate();

void MPU6050_Update(float dt);
