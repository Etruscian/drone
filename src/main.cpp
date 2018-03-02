#include <mbed.h>
#include <config.hpp>
#include "iniparser.h"
#include "transceiver.h"
#include "IMU.h"
#include "controller.hpp"
#include <iostream>
#include <bitset>

RawSerial serialConnection(USBTX,USBRX);
DigitalOut led(LED1), led2(LED2), led3(LED3), led4(LED4);
AnalogIn battery(p20);
dataStruct data;
configStruct config;
Transceiver radio(p5, p6, p7, p8, p9);
InterruptIn radioInterrupt(p10);
Ticker ticker;
Controller controller(p24, p22, p21, p23);
IMU imu;

uint8_t status;

void rxInterrupt(void){
    if (serialConnection.getc() == 'r')
        {
            radio.powerDown();
            __NVIC_SystemReset();
        }
}

void loadConfig(void){
    LocalFileSystem local("local");
    dictionary *dir = iniparser_load("/local/config.ini");

    // Read config for radio
    config.radioConfig.channel = iniparser_getint(dir, "radio:channel",101);
    config.radioConfig.txAddress = iniparser_getlongint(dir, "radio:txaddress",0x007FFFFFFF);
    config.radioConfig.rxAddress = iniparser_getlongint(dir, "radio:rxaddress",0x007FFFFFFF);
    config.radioConfig.transferSize = iniparser_getint(dir, "radio:transfersize",10);

    // Read config for ACRO mode
    char * keysAcro[9];
    const char ** keysAcroPtr = (const char **) &keysAcro;
    keysAcroPtr = iniparser_getseckeys(dir, "acromode", keysAcroPtr);
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Kp[i] = iniparser_getdouble(dir, (keysAcro[i]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Ki[i] = iniparser_getdouble(dir, (keysAcro[i+3]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Kd[i] = iniparser_getdouble(dir, (keysAcro[i+6]),0);
    }

    // Read config for stabilizing mode
    char * keysStabilizing[9];
    const char ** keysStabilizingPtr = (const char **) &keysStabilizing;
    keysStabilizingPtr = iniparser_getseckeys(dir, "stabilizingmode", keysStabilizingPtr);
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Kp[i] = iniparser_getdouble(dir, (keysStabilizing[i]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Ki[i] = iniparser_getdouble(dir, (keysStabilizing[i+3]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Kd[i] = iniparser_getdouble(dir, (keysStabilizing[i+6]),0);
    }

    // Read config for motor direction compensation
    char * keysSigns[12];
    const char ** keysSignsPtr = (const char **) &keysSigns;
    keysSignsPtr = iniparser_getseckeys(dir, "motordirections", keysSignsPtr);
    for (int i = 0; i<=3; i++){
        for (int j = 0; j<=2; j++){
            config.controllerConfig.signs[i][j] = iniparser_getdouble(dir, (keysSigns[3*i+j]),0);
        }
    }

    // Reading filter values
    config.imuconfig.itg3200.a = iniparser_getdouble(dir, "itg3200:a",0);
    config.imuconfig.itg3200.b = iniparser_getdouble(dir, "itg3200:b",0);
    config.imuconfig.itg3200.c = iniparser_getdouble(dir, "itg3200:c",0);

    config.imuconfig.hmc5883l.a = iniparser_getdouble(dir, "hmc5883l:a",0);
    config.imuconfig.hmc5883l.b = iniparser_getdouble(dir, "hmc5883l:b",0);
    config.imuconfig.hmc5883l.c = iniparser_getdouble(dir, "hmc5883l:c",0);

    config.imuconfig.adxl345.a = iniparser_getdouble(dir, "adxl345:a",0);
    config.imuconfig.adxl345.b = iniparser_getdouble(dir, "adxl345:b",0);
    config.imuconfig.adxl345.c = iniparser_getdouble(dir, "adxl345:c",0);

    config.tickerPeriod = (float) iniparser_getdouble(dir, "misc:tickerperiod",0.01);
    iniparser_freedict(dir);
}

void flight(void)
{   
    // radio.update();
    imu.update();
    if (data.remote.signalLost){
        data.remote.throttle = 0;
        led4 = 1;
    } else 
        led4 = 0;
    controller.update();
    data.batteryLevel.f = battery.read()*33.6247;
    // radio.setAcknowledgePayload(0);
}

void checkThrottleLow(void)
{
    // radio.update();
    if (data.remote.signalLost){
        data.remote.throttle = 0;
        led4 = 1;
    } else 
        led4 = 0;
    data.batteryLevel.f = battery.read()*33.6247;
    // radio.setAcknowledgePayload(0);

    if (data.remote.throttle <= 25)
    {
        ticker.detach();
        led3 = 0;
        ticker.attach(&flight, config.tickerPeriod);
    }
}

void checkThrottleHigh(void)
{
    // radio.update();
    if (data.remote.signalLost){
        data.remote.throttle = 0;
        led4 = 1;
    } else 
        led4 = 0;
    data.batteryLevel.f = battery.read()*33.6247;
    // radio.setAcknowledgePayload(0);

    // std::cout << data.remote.throttle << std::endl;
    if (data.remote.throttle >= 1000)
    {
        ticker.detach();
        led3 = 1;
        ticker.attach(&checkThrottleLow, config.tickerPeriod);
    }
}

void initialize(void)
{
    loadConfig();
    led = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;

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
        led2 = 1;
        return;
    }
    led4 = 1;
    while(!radio.firstPacketReceived){
        data.batteryLevel.f = battery.read()*33.6247;
        // std::cout << std::hex << config.radioConfig.txAddress << '\t' << std::hex << config.radioConfig.rxAddress << '\t' << std::dec << (uint16_t)config.radioConfig.channel << std::endl;
    }
        // radio.update();
    led4 = 0;
    controller.initialize(&data, &config.controllerConfig);
    ticker.attach(&checkThrottleHigh, config.tickerPeriod);

}

int main()
{
    serialConnection.attach(&rxInterrupt, Serial::RxIrq);
    initialize();
}
