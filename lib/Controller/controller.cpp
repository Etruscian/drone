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
    controllerMode = STABILIZE;
    updateParameters();
}

void Controller::update(void)
{
    if (!(*dataPtr).armMotor || ((*dataPtr).remote.throttle<=0.05 ))
    {
        escController[0].update(0);
        escController[1].update(0);
        escController[2].update(0);
        escController[3].update(0);
        return;
    }

    if ((*dataPtr).acroMode && (controllerMode == STABILIZE))
    {
        controllerMode = ACRO;
        updateParameters();
    }

    if (!(*dataPtr).acroMode && (controllerMode == ACRO))
    {
        controllerMode = STABILIZE;
        updateParameters();
    }

    if ((*dataPtr).newPacket){
        throttleRemote = (*dataPtr).remote.throttle;
        rollRemote = (*dataPtr).remote.roll;
        pitchRemote = (*dataPtr).remote.pitch;
        yawRemote = (*dataPtr).remote.yaw;
        (*dataPtr).newPacket = false;
    }

    throttle = throttle * 0.95 + throttleRemote * 0.05;
    roll = roll * 0.9 + rollRemote * 0.1;
    pitch = pitch * 0.9 + pitchRemote * 0.1;
    yaw = yaw * 0.9 + yawRemote * 0.1;

    if (controllerMode == STABILIZE)
    {
        rollError = roll - (*dataPtr).imu.roll;
        rollVelocityError = -(*dataPtr).imu.rollVelocity;
        pitchError = pitch - (*dataPtr).imu.pitch;
        pitchVelocityError = -(*dataPtr).imu.pitchVelocity;
    }

    if (controllerMode == ACRO)
    {
        rollError = roll - (*dataPtr).imu.rollVelocity;
        pitchError = pitch - (*dataPtr).imu.pitchVelocity;
    }

    yawError = yaw - (*dataPtr).imu.yawVelocity;

    for (int i = 0; i <= 3; i++)
    {
        rollControlValue = (*controllerConfigPtr).signs[i][0] * (Kp[0] * rollError + Kd[0] * rollVelocityError);
        pitchControlValue = (*controllerConfigPtr).signs[i][1] * (Kp[1] * pitchError + Kd[1] * pitchVelocityError);
        yawControlValue = (*controllerConfigPtr).signs[i][2] * Kp[2] * yawError;
        setpoint[i] = throttle * 125.0 + rollControlValue + pitchControlValue + yawControlValue;
        // escController[i].update(setpoint[i]);
        escController[i].update(throttle*125);
    }
}

void Controller::updateParameters(void)
{
    switch (controllerMode)
    {
    case ACRO:
        for (int i = 0; i <= 2; i++)
        {
            Kp[i] = (*controllerConfigPtr).acroModeConfig.Kp[i];
        }
        break;

    case STABILIZE:
        for (int i = 0; i <= 2; i++)
        {
            Kp[i] = (*controllerConfigPtr).stabilizingModeConfig.Kp[i];
        }
        break;
    }
}
