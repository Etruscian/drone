#include "IMU.h"
#include <iostream>

/*
Initializes the devices on the IMU. Returns an error code if an error occurs.
It starts with the GYRO, which needs no parameters.
Secondly, it initializes the accelerometer. This need a maximum range and 3
filter values.
Finally, the magnetometer is initialized.

@param          Pointer to the data struct
@return         status. 0 if success, error code otherwise

*/
int IMU::initialize(configStruct config, dataStruct *data)
{
    int status;
    // Initialize the GYRO.
    status = itg3200.initialize(config.imuconfig.itg3200);
    if (status)
        return status;

    // Initialize the accelerometer.
    status = adxl345.initialize(16, config.imuconfig.adxl345);
    if (status)
        return status | 0x10;

    // Initialize the magnetometer.
    // status = hmc5883l.initialize();
    // if (status)
    //     return status|0x20;

    dataPtr = data;
    _config = config;
    kalman.init();

    return 0;
}

/*
Calibrates the devices on the IMU.

@param          none
@return         none

*/
void IMU::calibrate(void)
{
    float valueRoll = 0, valuePitch = 0, valueYaw = 0;
    for (int i = 0; i < 10000; i++)
    {
        itg3200.read(&temp, &velocities[0], &velocities[1], &velocities[2]);
        valueRoll = valueRoll + (velocities[0] / 10000.0);
        valuePitch = valuePitch + (velocities[1] / 10000.0);
        valueYaw = valueYaw + (velocities[2] / 10000.0);
        wait_us(1000);
    }
    std::cout << (int32_t)(valueRoll * 1000000) << std::endl;
    std::cout << (int32_t)(valuePitch * 1000000) << std::endl;
    std::cout << (int32_t)(valueYaw * 1000000) << std::endl;
}

void IMU::updateGyro(void){
    itg3200.read(&temp, &velocities[0], &velocities[1], &velocities[2]);
    (*dataPtr).imu.rollVelocity = velocities[0];
    (*dataPtr).imu.pitchVelocity = velocities[1];
    (*dataPtr).imu.yawVelocity = velocities[2];
}

/*
Updates the dataPtr. 
It starts with updating the raw values.

@param          none
@return         none

*/
void IMU::updateAngles(void)
{
    adxl345.read(&accelerations[0], &accelerations[1], &accelerations[2]);
    estimator(&(*dataPtr).imu.roll, &(*dataPtr).imu.pitch);
}

void IMU::estimator(float *roll, float *pitch)
{
    *roll += (velocities[0]) / _config.tickerFrequency;
    *pitch += (velocities[1]) / _config.tickerFrequency;
    float rollAcc = atan2f(accelerations[1], accelerations[2]) * 180 / M_PI;
    float pitchAcc = atan2f(-accelerations[0], sqrt(accelerations[1] * accelerations[1] + accelerations[2] * accelerations[2])) * 180 / M_PI;

    *roll = *roll * 0.98 + rollAcc * 0.02;

    *pitch = *pitch * 0.98 + pitchAcc * 0.02;
}
