/*mpu6050register.h
宏定义常用的mpu6050寄存器地址
2019/6/4
张宇
*/
#include <stdint.h>
#define	SAMPLE_RATE		0x19	//陀螺仪采样率寄存器
#define	FILTER_CONFIG	0x1A	//低通滤波频率寄存器

#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围寄存器
#define	ACCEL_CONFIG	0x1C	//加速计自检寄存器、测量范围及低通滤波频率

#define MPU_ACCEL_XOUT_H 0x3B    //三个轴的加速度寄存器，高八位与低八位
#define MPU_ACCEL_XOUT_L 0x3C
#define MPU_ACCEL_YOUT_H 0x3D
#define MPU_ACCEL_YOUT_L 0x3E
#define MPU_ACCEL_ZOUT_H 0x3F
#define MPU_ACCEL_ZOUT_L 0x40

#define MPU_GYRO_XOUT_H 0x43   //三个轴的角速度寄存器，高八位与低八位
#define MPU_GYRO_XOUT_L 0x44
#define MPU_GYRO_YOUT_H 0x45
#define MPU_GYRO_YOUT_L 0x46
#define MPU_GYRO_ZOUT_H 0x47
#define MPU_GYRO_ZOUT_L 0x48

#define MPU_TEMP_H 0x41  //温度传感器寄存器，高八位与低八位
#define MPU_TEMP_L 0x42

#define MPU_POWER1 0x6B  //电源管理寄存器
#define MPU_POWER2 0x6C

typedef struct
{
    short AccelX,AccelY,AccelZ;
    short temprature;
    short GyroX,GyroY,GyroZ;

}GForceStruct;

void Get_Accel_Values(int handle, GForceStruct * Gdata);
void MPU6050_Dataanl(void);
void MPU6050Init();

extern int MPU6050_ACC_LAST_X,MPU6050_ACC_LAST_Y,MPU6050_ACC_LAST_Z;
extern int MPU6050_GYRO_LAST_X,MPU6050_GYRO_LAST_Y,MPU6050_GYRO_LAST_Z;
extern unsigned char GYRO_OFFSET_OK;
extern unsigned char ACC_OFFSET_OK;



