
#include <math.h>
#include "stm32f3xx_hal.h"
#include "mpu6050.h"

#define MPU6050_ADDR 0xD0 // Адрес I2C (0x68 << 1)
#define SMPLRT_DIV 0x19   // Sample Rate Divider
#define CONFIG 0x1A       // DLPF config
#define GYRO_CONFIG 0x1B  // Gyro Full Scale
#define ACCEL_CONFIG 0x1C // Accel Full Scale
#define PWR_MGMT_1 0x6B   // Power Management

#define BETA_FILTER 1.0f

#define GYRO_SENSITIVITY  0.0005326322180158476f // градусы*pi/180/сек/LSB
#define ACCEL_SENSITIVITY 0.0001220703125f    // g/LSB

// #define GYRO_SENSITIVITY  0.030487804878048783f // градусы*pi/180/сек/LSB
// #define ACCEL_SENSITIVITY 0.001220703125f    // g/LSB

extern I2C_HandleTypeDef hi2c2;

typedef struct
{
    int16_t Accel_X, Accel_Y, Accel_Z;
    int16_t Gyro_X, Gyro_Y, Gyro_Z;
    float Temperature;
} MPU6050_Data_Raw;

static MPU6050_Data_Raw offsets;
static MPU6050_Data_Raw mpu6050raw;
// static volatile uint8_t mpu_buf[14];

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static float ax = 0.f;
static float ay = 0.f;
static float az = -1.f;

MPU6050_Data mpu6050;

void MPU6050_Write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void MPU6050_Read(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, 0x3B, 1, (uint8_t *) mpu_buf, 14);
//}

void MPU6050_Init()
{
    HAL_Delay(1000);
    MPU6050_Write(PWR_MGMT_1, 0x00);   // Выход из спящего режима
    MPU6050_Write(SMPLRT_DIV, 0x00);   // Частота дискретизации = 1 кГц / (1+0) = 1000 Гц
    MPU6050_Write(CONFIG, 0x01);       // DLPF: полоса 184 Гц (для подавления шума)
    MPU6050_Write(GYRO_CONFIG, 0x10);  // Гироскоп: ±1000 °/с
    MPU6050_Write(ACCEL_CONFIG, 0x08); // Акселерометр: ±4g

//    HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, 0x3B, 1, (uint8_t *) mpu_buf, 14);
}

void MPU6050_Decode(MPU6050_Data_Raw *data, uint8_t *buf)
{
    data->Accel_X = (buf[0] << 8) | buf[1];
    data->Accel_Y = (buf[2] << 8) | buf[3];
    data->Accel_Z = (buf[4] << 8) | buf[5];
    data->Temperature = (buf[6] << 8) | buf[7];
    data->Gyro_X = (buf[8] << 8) | buf[9];
    data->Gyro_Y = (buf[10] << 8) | buf[11];
    data->Gyro_Z = (buf[12] << 8) | buf[13];
}

void MPU6050_Calibrate()
{
	const int32_t samples = 200;
	int32_t gx = 0, gy = 0, gz = 0;
	int32_t ax = 0, ay = 0, az = 0;

	for (int32_t i = 0; i < samples; ++i)
	{
		uint8_t buf[14];
		MPU6050_Read(0x3B, buf, 14);
		MPU6050_Decode(&offsets, (uint8_t *) buf);
		gx += offsets.Gyro_X;
		gy += offsets.Gyro_Y;
		gz += offsets.Gyro_Z;
		ax += offsets.Accel_X;
		ay += offsets.Accel_Y;
		az += offsets.Accel_Z;
		HAL_Delay(10);
	}
	offsets.Gyro_X = gx / samples;
	offsets.Gyro_Y = gy / samples;
	offsets.Gyro_Z = gz / samples;
	offsets.Accel_X = ax / samples;
	offsets.Accel_Y = ay / samples;
	offsets.Accel_Z = az / samples - 8192;
}

void Mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt);

void MPU6050_Update(float dt)
{
	uint8_t buf[14];
	MPU6050_Read(0x3B, buf, 14);
	MPU6050_Decode(&mpu6050raw, (uint8_t *) buf);

	mpu6050raw.Accel_X -= offsets.Accel_X;
	mpu6050raw.Accel_Y -= offsets.Accel_Y;
	mpu6050raw.Accel_Z -= offsets.Accel_Z;
	mpu6050raw.Gyro_X -= offsets.Gyro_X;
	mpu6050raw.Gyro_Y -= offsets.Gyro_Y;
	mpu6050raw.Gyro_Z -= offsets.Gyro_Z;

	Mahony(mpu6050raw.Gyro_X * GYRO_SENSITIVITY, mpu6050raw.Gyro_Y * GYRO_SENSITIVITY, mpu6050raw.Gyro_Z * GYRO_SENSITIVITY,
			mpu6050raw.Accel_X * ACCEL_SENSITIVITY, mpu6050raw.Accel_Y * ACCEL_SENSITIVITY, mpu6050raw.Accel_Z * ACCEL_SENSITIVITY, dt);
}

inline void Mahony(float gx, float gy, float gz, float _ax, float _ay, float _az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    ax = ax * 0.8f + _ax * 0.2f;
    ay = ay * 0.8f + _ay * 0.2f;
    az = az * 0.8f + _az * 0.2f;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        gx += BETA_FILTER * halfex;
        gy += BETA_FILTER * halfey;
        gz += BETA_FILTER * halfez;
    }

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

//    mpu6050.vx = 2.0f * (q1 * q3 - q0 * q2);
//    mpu6050.vy = 1.0f - 2.0f * (q0 * q0 + q3 * q3);
  //  mpu6050.vx = 2.0f * (q0 * q2 + q3 * q1);
//    mpu6050.vy = 2.0f * (q1 * q2 - q3 * q0);
    mpu6050.vx = -2.0f * (q0 * q1 + q2 * q3);
    mpu6050.vy = 2.0f * (q1 * q3 - q0 * q2);
    //    mpu6050.vy = 1.0f - 2.0f * (q0 * q0 + q1 * q1);
  //  mpu6050.vx = 2.0f * (q0 * q1 - q3 * q2);
//    mpu6050.vy = 2.0f * (q3 * q0 + q1 * q2);

//    mpu6050.vx = -ax;
  //  mpu6050.vy = -ay;

    /*
    mpu6050.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
    	mpu6050.pitch = copysignf(M_PI / 2.0f, sinp);
    else
    	mpu6050.pitch = asinf(sinp);

    mpu6050.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    */
}
