#include <mbed.h>
#include "Watchdog.hpp"
#include <config.hpp>
#include "iniparser.h"
#include "serialHandler.hpp"
#include "transceiver.h"
#include "IMU.h"
#include "controller.hpp"

Watchdog watchdog;

Serial pc(USBTX,USBRX);
SerialHandler serial;
DigitalOut led(LED1), led2(LED2), led3(LED3), led4(LED4), radioPower(p30);
AnalogIn battery(p20);
dataStruct data;
configStruct config;
Transceiver radio(p5, p6, p7, p8, p9);
InterruptIn radioInterrupt(p10);
Ticker controllerInterrupt, imuInterrupt, ledTicker;
Controller controller(p24, p22, p21, p23);
IMU imu;

uint8_t status;

void loadConfig(void){
    LocalFileSystem local("local");
    dictionary *dir = iniparser_load("/local/config.ini");

    // Read config for radio
    config.radioConfig.channel = iniparser_getint(dir, "radio:channel",101);
    config.radioConfig.txAddress = iniparser_getlongint(dir, "radio:txaddress",0x007DEADBEE);
    config.radioConfig.rxAddress = iniparser_getlongint(dir, "radio:rxaddress",0x007DEADBEE);
    config.radioConfig.transferSize = iniparser_getint(dir, "radio:transfersize",17);

    // Read remote prescalers
    config.controllerConfig.prescaler[0] = (float)iniparser_getdouble(dir, "controller:prescaler_roll",0);
    config.controllerConfig.prescaler[1] = (float)iniparser_getdouble(dir, "controller:prescaler_pitch",0);
    config.controllerConfig.prescaler[2] = (float)iniparser_getdouble(dir, "controller:prescaler_yaw",0);
    config.controllerConfig.imuPrescaler[0] = (float)iniparser_getdouble(dir, "controller:prescaler_roll_imu",0);
    config.controllerConfig.imuPrescaler[1] = (float)iniparser_getdouble(dir, "controller:prescaler_pitch_imu",0);
    config.controllerConfig.imuPrescaler[2] = (float)iniparser_getdouble(dir, "controller:prescaler_yaw_imu",0);


    // Read config for ACRO mode
    char * keysAcro[9];
    const char ** keysAcroPtr = (const char **) &keysAcro;
    keysAcroPtr = iniparser_getseckeys(dir, "acromode", keysAcroPtr);
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Kp[i] = (float)iniparser_getdouble(dir, (keysAcro[i]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Ki[i] = (float)iniparser_getdouble(dir, (keysAcro[i+3]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Kd[i] = (float)iniparser_getdouble(dir, (keysAcro[i+6]),0);
    }

    // Read config for stabilizing mode
    char * keysStabilizing[9];
    const char ** keysStabilizingPtr = (const char **) &keysStabilizing;
    keysStabilizingPtr = iniparser_getseckeys(dir, "stabilizingmode", keysStabilizingPtr);
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Kp[i] = (float)iniparser_getdouble(dir, (keysStabilizing[i]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Ki[i] = (float)iniparser_getdouble(dir, (keysStabilizing[i+3]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Kd[i] = (float)iniparser_getdouble(dir, (keysStabilizing[i+6]),0);
    }

    // Read config for motor direction compensation
    char * keysSigns[12];
    const char ** keysSignsPtr = (const char **) &keysSigns;
    keysSignsPtr = iniparser_getseckeys(dir, "motordirections", keysSignsPtr);
    for (int i = 0; i<=3; i++){
        for (int j = 0; j<=2; j++){
            config.controllerConfig.signs[i][j] = iniparser_getint(dir, (keysSigns[3*i+j]),0);
        }
    }

    // Reading filter values
    config.imuconfig.itg3200.a = (float)iniparser_getdouble(dir, "itg3200:a",0);
    config.imuconfig.itg3200.b = (float)iniparser_getdouble(dir, "itg3200:b",0);
    config.imuconfig.itg3200.c = (float)iniparser_getdouble(dir, "itg3200:c",0);

    config.imuconfig.hmc5883l.a = (float)iniparser_getdouble(dir, "hmc5883l:a",0);
    config.imuconfig.hmc5883l.b = (float)iniparser_getdouble(dir, "hmc5883l:b",0);
    config.imuconfig.hmc5883l.c = (float)iniparser_getdouble(dir, "hmc5883l:c",0);

    config.imuconfig.adxl345.a = (float)iniparser_getdouble(dir, "adxl345:a",0);
    config.imuconfig.adxl345.b = (float)iniparser_getdouble(dir, "adxl345:b",0);
    config.imuconfig.adxl345.c = (float)iniparser_getdouble(dir, "adxl345:c",0);

    config.tickerFrequency = (uint16_t) iniparser_getint(dir, "misc:tickerfrequency",500);
    iniparser_freedict(dir);
}

void imuUpdate(void){
    imu.update();
}

void flight(void)
{   
    // watchdog.kick();
    imu.update();
    controller.update();
    data.batteryLevel.f = battery.read()*3.3*(81.6+476)/81.6;
}

void ledUpdate(void){
    led4 = 1-led4;
}

void initialize(void)
{
    loadConfig();
    led = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;
    serial.initialize(&data, &config); 
    radioPower=1;
    wait(1);
    status = radio.initialize(config, &data);
    if (status)
    {
        led = 1;
        return;
    }
    radioInterrupt.fall(callback(&radio, &Transceiver::interruptHandler));
    status = imu.initialize(config, &data);
    if (status)
    {
        pc.printf("%u\n", status);
        led2 = 1;
        return;
    }

    ledTicker.attach(&ledUpdate, 0.5);
    while(!data.newPacket){
        data.batteryLevel.f = battery.read()*3.3*(81.6+476)/81.6;
    }
    ledTicker.detach();
    led4 = 0;
    
    controller.initialize(&data, &config.controllerConfig);
    controllerInterrupt.attach(&flight, 1.0/config.tickerFrequency);
    imuInterrupt.attach(callback(&imu, &IMU::update),0.001);
    // watchdog.kick(0.1);
}

int main()
{
    initialize();
}
