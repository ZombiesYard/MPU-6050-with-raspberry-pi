#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <sys/time.h>
#include <termios.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <math.h>
#include <unistd.h>
#include "mpu6050register.h"
#include "quaternion.h"
GForceStruct Data;  //声明结构体为新的数据类型方便使用

float Ax,Ay,Az;
float Angel_accX,Angel_accY,Angel_accZ;
const float_t Acceleration_G_Factor=16.0/32768.0;
const float_t Acceleration_Factor=2.0/32768.00;
const float_t Angel_Acceleration_Factor=180.0/3.14;

int fd;
int16_t temp,temp1,acceleration,xaccel,yaccel,zaccel;
int i2c_handle;

int ACC_OFFSET_X,ACC_OFFSET_Y,ACC_OFFSET_Z;
int GYRO_OFFSET_X,GYRO_OFFSET_Y,GYRO_OFFSET_Z;

unsigned char	GYRO_OFFSET_OK = 1;
unsigned char	ACC_OFFSET_OK = 1;

int MPU6050_ACC_LAST_X,MPU6050_ACC_LAST_Y,MPU6050_ACC_LAST_Z;
int MPU6050_GYRO_LAST_X,MPU6050_GYRO_LAST_Y,MPU6050_GYRO_LAST_Z;

void MPU6050Init()
{
    usleep(100000);//在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    i2c_smbus_write_byte_data(fd,MPU_POWER1,0x00);//解除休眠状态
    i2c_smbus_write_byte_data(fd,SAMPLE_RATE,0x07);//设置陀螺仪采样率，125Hz
    i2c_smbus_write_byte_data(fd,FILTER_CONFIG,0x06);//低通滤波器的设置 5Hz
    i2c_smbus_write_byte_data(fd,GYRO_CONFIG,0x18);//陀螺仪自检、测量范围，典型值：0x18(不自检，2000deg/s)
    i2c_smbus_write_byte_data(fd,ACCEL_CONFIG,0x01);//加速计自检、测量范围及低通滤波频率，典型值：0x01(不自检，2G，5Hz)
}

void Get_Accel_Values(int handle,GForceStruct * GData)
{
    GData->temprature = i2c_smbus_read_byte_data(fd, MPU_TEMP_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_TEMP_L);//获取传感器的温度
    GData->AccelX = i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT_L);//获取加速度x
    GData->AccelY = i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT_L);//获取加速度y
    GData->AccelZ = i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT_L);//获取加速度z
    GData->GyroX = i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT_L);//获取角速度x
    GData->GyroY = i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT_L);//获取角速度y
    GData->GyroZ = i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT_H) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT_L);//获取角速度z
}

void MPU6050_Dataanl(void)//函数功能：MPU6050数据读取并处理
{
    MPU6050_ACC_LAST_X = Data.AccelX - ACC_OFFSET_X;
    MPU6050_ACC_LAST_Y = Data.AccelY - ACC_OFFSET_Y;
    MPU6050_ACC_LAST_Z = Data.AccelZ - ACC_OFFSET_Z;

    MPU6050_GYRO_LAST_X = Data.GyroX - GYRO_OFFSET_X;
    MPU6050_GYRO_LAST_Y = Data.GyroY - GYRO_OFFSET_Y;
    MPU6050_GYRO_LAST_Z = Data.GyroZ - GYRO_OFFSET_Z;

    if (!GYRO_OFFSET_OK)
    {
        static long int tempgx = 0,tempgy = 0,tempgz = 0;
        static unsigned char cnt_g = 0;

        if (cnt_g == 0)
        {
            GYRO_OFFSET_X=0;
            GYRO_OFFSET_Y=0;
            GYRO_OFFSET_Z=0;
            tempgx = 0;
            tempgy = 0;
            tempgz = 0;
            cnt_g = 1;
        }

        tempgx += MPU6050_GYRO_LAST_X;
        tempgy += MPU6050_GYRO_LAST_Y;
        tempgz += MPU6050_GYRO_LAST_Z;

        if (cnt_g == 200)
        {
            GYRO_OFFSET_X = tempgx/cnt_g;
            GYRO_OFFSET_Y = tempgy/cnt_g;
            GYRO_OFFSET_Z = tempgz/cnt_g;
            cnt_g = 0;
            GYRO_OFFSET_OK = 1;
        }
        
        cnt_g ++;   
    }

    if (!ACC_OFFSET_OK)
    {
        static long int tempax=0,tempay=0,tempaz=0;
        static unsigned char cnt_a=0;

        if (cnt_a == 0)
        {
            ACC_OFFSET_X = 0;
            ACC_OFFSET_Y = 0;
            ACC_OFFSET_Z = 0;
            tempax = 0;
            tempay = 0;
            tempaz = 0;
            cnt_a = 1;
        }

        tempax += MPU6050_ACC_LAST_X;//累加
        tempay += MPU6050_ACC_LAST_Y;
        tempaz += MPU6050_ACC_LAST_Z;

        if (cnt_a == 200)
        {
            ACC_OFFSET_X = tempax/cnt_a;
            ACC_OFFSET_Y = tempay/cnt_a;
            ACC_OFFSET_Z = tempaz/cnt_a;
            cnt_a = 0;
            ACC_OFFSET_OK = 1;
        }
        cnt_a ++;
    }
    
}

int main(int argc,char **argv)
{
    float_t xangel,yangel,zangel;
    char *fileName = "/dev/i2c-1";
    int  address = 0x68;
    
    if ((fd = open(fileName, O_RDWR)) < 0)
    {
        printf("Failed to open i2c port\n");
        exit(1);
    }
    
    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        printf("Unable to get bus access to talk to slave\n");
        exit(1);
    }

    MPU6050Init();

    while (1) {

        Get_Accel_Values(i2c_handle,&Data);
        Prepare_Data();
        QuaternionPrepare();
        printf("温度:%+4.1f X轴加速度:%+6.2f Y轴加速度:%+6.2f Z轴加速度:%+6.2f X轴夹角:%f Y轴夹角:%f Z轴夹角:%f\n",Data.temprature/340.0f+36.53, Data.AccelX*Acceleration_Factor, Data.AccelY*Acceleration_Factor, Data.AccelZ*Acceleration_Factor, Q_ANGLE_X, Q_ANGLE_Y, Q_ANGLE_Z);

    
    }
		
    return 0;
}
