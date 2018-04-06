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
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
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

    throttle = throttle * (1-0.1) + throttleRemote * 0.1;
    roll = roll * (1-0.1) + rollRemote * 0.1;
    pitch = pitch * (1-0.1) + pitchRemote * 0.1;
    yaw = yaw * (1-0.1) + yawRemote * 0.1;
    // std::cout << (int16_t)(pidRoll.calculate(roll, (*dataPtr).imu.roll*(*controllerConfigPtr).imuPrescaler[0])*1000) << '\t' <<(int16_t)(pidPitch.calculate(pitch, (*dataPtr).imu.pitch*(*controllerConfigPtr).imuPrescaler[1])*1000) <<std::endl;

    for (int i = 0; i <= 3; i++)
    {
        float setpoint = throttle * 125 +  (*controllerConfigPtr).signs[i][0] * pidRoll.calculate(roll, (*dataPtr).imu.roll*(*controllerConfigPtr).imuPrescaler[0]) + \
                                            (*controllerConfigPtr).signs[i][1] * pidPitch.calculate(pitch, (*dataPtr).imu.pitch*(*controllerConfigPtr).imuPrescaler[1]) + \
                                            (*controllerConfigPtr).signs[i][2] * pidYaw.calculate(yaw, (*dataPtr).imu.yawVelocity*(*controllerConfigPtr).imuPrescaler[2]);
        escController[i].update(setpoint);
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
            Ki[i] = (*controllerConfigPtr).acroModeConfig.Ki[i];
            Kd[i] = (*controllerConfigPtr).acroModeConfig.Kd[i];
        }
        pidRoll.initialize(Kp[0], Ki[0], Kd[0], 0.001, 50.0, -50.0);
        pidPitch.initialize(Kp[1], Ki[1], Kd[1], 0.001, 50.0, -50.0);
        pidYaw.initialize(Kp[2], Ki[2], Kd[2], 0.001, 50.0, -50.0);
        break;

    case STABILIZE:
        for (int i = 0; i <= 2; i++)
        {
            Kp[i] = (*controllerConfigPtr).stabilizingModeConfig.Kp[i];
            Ki[i] = (*controllerConfigPtr).stabilizingModeConfig.Ki[i];
            Kd[i] = (*controllerConfigPtr).stabilizingModeConfig.Kd[i];
        }
        pidRoll.initialize(Kp[0], Ki[0], Kd[0], 0.001, 50.0, -50.0);
        pidPitch.initialize(Kp[1], Ki[1], Kd[1], 0.001, 50.0, -50.0);
        pidYaw.initialize(Kp[2], Ki[2], Kd[2], 0.001, 50.0, -50.0);
        break;
    }
}
