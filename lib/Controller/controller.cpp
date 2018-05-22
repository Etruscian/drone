#include "controller.hpp"
#include <iostream>

void Controller::initialize(dataStruct *data, controllerConfigStruct *controllerConfig)
{
    escController[0].initialize();
    escController[1].initialize();
    escController[2].initialize();
    escController[3].initialize();
    dataPtr = data;
    controllerConfigPtr = controllerConfig;

    setpoint[0] = 0;
    setpoint[1] = 0;
    setpoint[2] = 0;
    setpoint[3] = 0;
    updateParameters();
}

void Controller::update(void)
{
    if ((*dataPtr).newPacket)
    {
        throttleRemote = (*dataPtr).remote.throttle;
        rollRemote = (*dataPtr).remote.roll;
        pitchRemote = (*dataPtr).remote.pitch;
        yawRemote = (*dataPtr).remote.yaw;
        (*dataPtr).newPacket = false;
    }

    throttle = throttle * (1 - 0.1) + throttleRemote * 0.1;
    roll = roll * (1 - 0.1) + rollRemote * 0.1;
    pitch = pitch * (1 - 0.1) + pitchRemote * 0.1;
    yaw = yaw * (1 - 0.1) + yawRemote * 0.1;

    positionController();

    velocityController();
}

void Controller::positionController(void)
{

    if ((*dataPtr).acroMode)
    {
        velocitySetpoint[0] = roll;
        velocitySetpoint[1] = pitch;
        return;
    }

    velocitySetpoint[0] = pidRoll.calculate(roll, (*dataPtr).imu.roll * (*controllerConfigPtr).imuPrescaler[0]);
    velocitySetpoint[1] = pidPitch.calculate(pitch, (*dataPtr).imu.pitch * (*controllerConfigPtr).imuPrescaler[1]);
}

void Controller::velocityController(void)
{
    for (int i = 0; i <= 3; i++)
    {
        float setpoint = (*controllerConfigPtr).signs[i][0] * pidRollVelocity.calculate(velocitySetpoint[0], (*dataPtr).imu.rollVelocity * (*controllerConfigPtr).imuPrescaler[0]) +
                         (*controllerConfigPtr).signs[i][1] * pidPitchVelocity.calculate(velocitySetpoint[1], (*dataPtr).imu.pitchVelocity * (*controllerConfigPtr).imuPrescaler[1]) +
                         (*controllerConfigPtr).signs[i][2] * pidYawVelocity.calculate(yaw, (*dataPtr).imu.yawVelocity * (*controllerConfigPtr).imuPrescaler[2]);

        escController[i].update(throttle * 125.0 + setpoint);
    }
}

void Controller::updateParameters(void)
{

    for (int i = 0; i <= 2; i++)
    {
        Kp[i] = (*controllerConfigPtr).stabilizingModeConfig.Kp[i];
        Ki[i] = (*controllerConfigPtr).stabilizingModeConfig.Ki[i];
        Kd[i] = (*controllerConfigPtr).stabilizingModeConfig.Kd[i];
    }
    pidRoll.initialize(Kp[0], Ki[0], 0, 0.001, 50.0, -50.0);
    pidRollVelocity.initialize(Kd[0], 0, 0, 0.001, 50.0, -50.0);
    pidPitch.initialize(Kp[1], Ki[1], 0, 0.001, 50.0, -50.0);
    pidPitchVelocity.initialize(Kd[1], 0, 0, 0.001, 50.0, -50.0);
    pidYawVelocity.initialize(Kp[2], Ki[2], Kd[2], 0.001, 50.0, -50.0);
}
