#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

struct remoteStruct
{
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t missedPackets;

    remoteStruct() : throttle(0),
                     roll(0),
                     pitch(0),
                     yaw(0)
    {
    }
};

struct imuStruct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t rollVelocity;
    int16_t pitchVelocity;
    int16_t yawVelocity;
};

struct dataStruct
{
    remoteStruct remote;
    imuStruct imu;
    uint16_t batteryLevel;
    bool acroMode;
    bool armMotor;

    dataStruct() : acroMode(true),
                   armMotor(false)
    {
    }
};

struct radioConfigStruct
{
    uint8_t channel;
    uint64_t txAddress;
    uint64_t rxAddress;
    uint8_t transferSize;

    radioConfigStruct() : channel(101),
                          txAddress(0x00F0F0F0F0),
                          rxAddress(0x00F0F0F0F0),
                          transferSize(8)
    {
    }
};

struct acroModeStruct
{
    float Kp[4][3], Ki[4][3], Kd[4][3];
};

struct stabilizingModeStruct
{
    float Kp[4][3], Ki[4][3], Kd[4][3];
};

struct controllerConfigStruct
{
    acroModeStruct acroModeConfig;
    stabilizingModeStruct stabilizingModeConfig;
    int8_t signs[4][3];

    controllerConfigStruct() : signs{{1, 1, 1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, 1}}
    {
    }
};

struct ITG3200ConfigStruct
    {
        float a = 0.01;
        float b = 0.01;
        float c = 0.01;
    };

struct HMC5883LConfigStruct
    {
        float a = 0.001;
        float b = 0.001;
        float c = 0.001;
    };

struct AXDL345ConfigStruct
    {
        float a = 0.001;
        float b = 0.001;
        float c = 0.001;
    };

struct imuConfigStruct
{
    AXDL345ConfigStruct adxl345;

    HMC5883LConfigStruct hmc5883l;

    ITG3200ConfigStruct itg3200;
};

struct configStruct
{
    radioConfigStruct radioConfig;
    controllerConfigStruct controllerConfig;
    imuConfigStruct imuconfig;
    float tickerPeriod;

    configStruct() : tickerPeriod(1.0 / 500.0)
    {
    }
};

#endif
