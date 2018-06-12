#include "controller.hpp"
#include <iostream>

void Controller::initialize(void)
{
    escController[0].initialize();
    escController[1].initialize();
    escController[2].initialize();
    escController[3].initialize();

    setpoint[0] = 0;
    setpoint[1] = 0;
    setpoint[2] = 0;
    setpoint[3] = 0;
    
    for (int i = 0; i <= 2; i++)
    {
        Kp[i] = config.controllerConfig.stabilizingModeConfig.Kp[i];
        Ki[i] = config.controllerConfig.stabilizingModeConfig.Ki[i];
        Kd[i] = config.controllerConfig.stabilizingModeConfig.Kd[i];
    }
    pidRoll.initialize(Kp[0], Ki[0], 0, 0.001, 50.0, -50.0);
    pidRollVelocity.initialize(Kd[0], 0, 0, 0.001, 50.0, -50.0);
    pidPitch.initialize(Kp[1], Ki[1], 0, 0.001, 50.0, -50.0);
    pidPitchVelocity.initialize(Kd[1], 0, 0, 0.001, 50.0, -50.0);
    pidYawVelocity.initialize(Kp[2], Ki[2], Kd[2], 0.001, 50.0, -50.0);

}

void Controller::update(void)
{
    if (data.newPacket)
    {
        throttleRemote = data.remote.throttle;
        rollRemote = data.remote.roll;
        pitchRemote = data.remote.pitch;
        yawRemote = data.remote.yaw;
        data.newPacket = false;
    }

    throttle = throttle * (1 - (config.flightTickerFrequency/100.0)) + throttleRemote * (config.flightTickerFrequency/100.0);
    roll = roll * (1 - (config.flightTickerFrequency/100.0)) + rollRemote * (config.flightTickerFrequency/100.0);
    pitch = pitch * (1 - (config.flightTickerFrequency/100.0)) + pitchRemote * (config.flightTickerFrequency/100.0);
    yaw = yaw * (1 - (config.flightTickerFrequency/100.0)) + yawRemote * (config.flightTickerFrequency/100.0);

    if (data.acroMode){
        velocitySetpoint[0] = roll;
        velocitySetpoint[1] = pitch;
    } else {
        positionController();
    }

    velocityController();
    updateMotorControllers();
}

void Controller::positionController(void)
{
    velocitySetpoint[0] = pidRoll.calculate(roll, data.imu.roll * config.controllerConfig.imuPrescaler[0]);
    velocitySetpoint[1] = pidPitch.calculate(pitch, data.imu.pitch * config.controllerConfig.imuPrescaler[1]);
}

void Controller::velocityController(void)
{
    for (int i = 0; i <= 3; i++)
    {
        setpoint[i] = config.controllerConfig.signs[i][0] * pidRollVelocity.calculate(velocitySetpoint[0],  data.imu.rollVelocity * config.controllerConfig.imuPrescaler[0]) +
                      config.controllerConfig.signs[i][1] * pidPitchVelocity.calculate(velocitySetpoint[1],  data.imu.pitchVelocity * config.controllerConfig.imuPrescaler[1]) +
                      config.controllerConfig.signs[i][2] * pidYawVelocity.calculate(yaw,  data.imu.yawVelocity * config.controllerConfig.imuPrescaler[2]);
    }
}

void Controller::updateMotorControllers(void){
    for (int i=0; i<=3; i++){
        escController[i].update(throttle * 125.0 + setpoint[i]);
        std::cout << (uint16_t)(setpoint[i]) << std::endl;}
}
