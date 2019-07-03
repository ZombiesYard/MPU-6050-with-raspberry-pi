#include <math.h>
#include "quaternion.h"
#include "mpu6050register.h"

#define RtA 	   57.324841  //弧度到角度
#define AtR    	   0.0174533  //度到角度
#define Acc_G 	   0.0011963  //加速度变成G
#define Gyro_G 	   0.0152672  //角速度变成度
#define Gyro_Gr	   0.0002663  //
#define FILTER_NUM 20 //滑动窗口的采样值个数
#define Kp 10.0f        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f       // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f    // half the sample period采样周期的一半

int   ACC_AVG_X,ACC_AVG_Y,ACC_AVG_Z;      //平均值滤波后的ACC
float GYRO_I_X,GYRO_I_Y,GYRO_I_Z;         //陀螺仪积分
float EXP_ANGLE_X,EXP_ANGLE_Y,EXP_ANGLE_Z;//期望角度
float DIF_ANGLE_X,DIF_ANGLE_Y,DIF_ANGLE_Z;//期望角度和实际角度的差
float Q_ANGLE_X,Q_ANGLE_Y,Q_ANGLE_Z,Q_ANGLE_Yaw;      //四元数计算出的角度
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
int ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	//加速度滑动窗口滤波数组

void Prepare_Data(void)
{
    static unsigned char filter_cnt=0;
    long int temp1=0,temp2=0,temp3=0;
    unsigned char i;
    MPU6050_Dataanl();//完成传感器数据的读取和计算，并且对数据简单处理
    ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST_X;
    ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST_Y;
    ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST_Z;
    for ( i = 0; i < FILTER_NUM; i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
        temp3 += ACC_Z_BUF[i];
    }
    ACC_AVG_X = temp1 / FILTER_NUM;
    ACC_AVG_Y = temp2 / FILTER_NUM;
    ACC_AVG_Z = temp3 / FILTER_NUM;
    filter_cnt ++;
    if(filter_cnt == FILTER_NUM) filter_cnt = 0;
    GYRO_I_X += MPU6050_GYRO_LAST_X * Gyro_G * 0.0001;
    GYRO_I_Y += MPU6050_GYRO_LAST_Y * Gyro_G * 0.0001;
    GYRO_I_Z += MPU6050_GYRO_LAST_Z * Gyro_G * 0.0001;
}

void QuaternionPrepare(void)
{
    SolutionQuaternion(MPU6050_GYRO_LAST_X * Gyro_Gr, MPU6050_GYRO_LAST_Y * Gyro_Gr, MPU6050_GYRO_LAST_Z * Gyro_Gr, ACC_AVG_X, ACC_AVG_Y, ACC_AVG_Z);
}

void SolutionQuaternion(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx,vy,vz;
    float ex,ey,ez;

    float q0q0 = q0 * q0; //先把矩阵定义好方便套公式
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    if(ax*ay*az == 0) return;

    norm = sqrt(ax * ax+ay * ay+az * az);//acc数据归一化
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    vx = 2 * (q1q3 - q0q2); //估计重力方向和流量/变迁
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = (ay * vz - az * vy); //向量外积在相减得到差分就是误差
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    exInt = exInt + ex * Ki; //对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp*ex + exInt; //将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt; //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或者自减  

    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT; //四元数的微分方程
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3); //化简
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    Q_ANGLE_Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3;
    Q_ANGLE_Y = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3;
    Q_ANGLE_X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;

}