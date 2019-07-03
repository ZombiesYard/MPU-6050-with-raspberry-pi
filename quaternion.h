#ifndef _IMU_H_
#define _IMU_H_

//#include "mpu6050register.h"

extern float Q_ANGLE_X,Q_ANGLE_Y,Q_ANGLE_Z;

void Prepare_Data(void);
void SolutionQuaternion(float gx, float gy, float gz, float ax, float ay, float az);
void QuaternionPrepare(void);
//extern void MPU6050_Dataanl(void);

#endif
