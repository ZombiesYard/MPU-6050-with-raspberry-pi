
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <math.h>
#include <unistd.h>


#define MPU_ACCEL_XOUT1 0x3b
#define MPU_ACCEL_XOUT2 0x3c
#define MPU_ACCEL_YOUT1 0x3d
#define MPU_ACCEL_YOUT2 0x3e
#define MPU_ACCEL_ZOUT1 0x3f
#define MPU_ACCEL_ZOUT2 0x40

#define MPU_GYRO_XOUT1 0x43
#define MPU_GYRO_XOUT2 0x44
#define MPU_GYRO_YOUT1 0x45
#define MPU_GYRO_YOUT2 0x46
#define MPU_GYRO_ZOUT1 0x47
#define MPU_GYRO_ZOUT2 0x48

#define MPU_TEMP1 0x41
#define MPU_TEMP2 0x42

#define MPU_POWER1 0x6b
#define MPU_POWER2 0x6c

float Ax,Ay,Az;
float Angel_accX,Angel_accY,Angel_accZ;

int main(int argc, char **argv)
{
    int fd;
    const float Acceleration_Factor=1.0/16384.0;
    const float Angel_Acceleration_Factor=180.0/3.14;

    char *fileName = "/dev/i2c-1";
    int  address = 0x68;
	
    if ((fd = open(fileName, O_RDWR)) < 0) {
        printf("Failed to open i2c port\n");
        exit(1);
    }
	
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        printf("Unable to get bus access to talk to slave\n");
        exit(1);
    }
    
    int8_t power = i2c_smbus_read_byte_data(fd, MPU_POWER1);
    i2c_smbus_write_byte_data(fd, MPU_POWER1, ~(1 << 6) & power);

    while (1) {
        usleep(100000);
        int16_t temp = i2c_smbus_read_byte_data(fd, MPU_TEMP1) << 8 |
                        i2c_smbus_read_byte_data(fd, MPU_TEMP2);

        int16_t xaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT1) << 8 |
                         i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT2);
        int16_t yaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT1) << 8 |
                         i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT2);
        int16_t zaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT1) << 8 |
                         i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT2);

        int16_t xgyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT1) << 8 |
                        i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT2);
        int16_t ygyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT1) << 8 |
                        i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT2);
        int16_t zgyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT1) << 8 |
                        i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT2);
        Ax=xaccel*Acceleration_Factor;
        Ay=yaccel*Acceleration_Factor;
        Az=zaccel*Acceleration_Factor;
        Angel_accX=acos(Ax/(sqrt(Ax*Ax+Az*Az+Ay*Ay)))*Angel_Acceleration_Factor;
        Angel_accY=acos(Ay/(sqrt(Ax*Ax+Ay*Ay+Az*Az)))*Angel_Acceleration_Factor;
        Angel_accZ=acos(Az/(sqrt(Ax*Ax+Ay*Ay+Az*Az)))*Angel_Acceleration_Factor;

        printf("temp: %f\n", (float)temp / 340.0f + 36.53);
        printf("accel x,y,z: %d, %d, %d\n", (int)xaccel, (int)yaccel, (int)zaccel);
        printf("gyro x,y,z: %d, %d, %d\n\n", (int)xgyro, (int)ygyro, (int)zgyro);
        printf("Angel x,y,z: %d, %d, %d\n\n\n",(int)Angel_accX, (int)Angel_accY,(int)Angel_accZ);
    
    }
		
    return 0;
}
