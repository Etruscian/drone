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
    if (!(*dataPtr).armMotor || ((*dataPtr).remote.throttle<=10 ))
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

    if (controllerMode == STABILIZE)
    {
        rollError = (*dataPtr).remote.roll/10 - (*dataPtr).imu.roll;
        pitchError = (*dataPtr).remote.pitch/10 - (*dataPtr).imu.pitch;
    }

    if (controllerMode == ACRO)
    {
        rollError = (*dataPtr).remote.roll - (*dataPtr).imu.rollVelocity;
        pitchError = (*dataPtr).remote.pitch - (*dataPtr).imu.pitchVelocity;
    }

    yawError = (*dataPtr).remote.yaw - (*dataPtr).imu.yawVelocity/100;

    for (int i = 0; i <= 3; i++)
    {
        rollControlValue = (*controllerConfigPtr).signs[i][0] * Kp[i][0] * rollError;
        pitchControlValue = (*controllerConfigPtr).signs[i][1] * Kp[i][1] * pitchError;
        yawControlValue = (*controllerConfigPtr).signs[i][2] * Kp[i][2] * yawError;
        setpoint[i] = (*dataPtr).remote.throttle / 1023.0 * 125.0 + rollControlValue + pitchControlValue + yawControlValue;
    }

    for (int i = 0; i <= 3; i++)
    {
        escController[i].update(setpoint[i]);
    }

    // uint8_t value = (uint8_t)((*dataPtr).remote.throttle / 1023.0 * 125.0); // - (*pitch)/90.0*45.0;
    // escController[0].update(value);
    // escController[1].update(value);
    // escController[2].update(value);
    // escController[3].update(value);
}

void Controller::updateParameters(void)
{
    switch (controllerMode)
    {
    case ACRO:
        for (int i = 0; i <= 3; i++)
        {
            for (int j = 0; j <= 2; j++)
            {
                Kp[i][j] = (*controllerConfigPtr).acroModeConfig.Kp[i][j];
            }
        }
        break;

    case STABILIZE:
        for (int i = 0; i <= 3; i++)
        {
            for (int j = 0; j <= 2; j++)
            {
                Kp[i][j] = (*controllerConfigPtr).stabilizingModeConfig.Kp[i][j];
            }
        }
        break;
    }
}
