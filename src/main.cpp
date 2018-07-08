#include <mbed.h>
#include "Watchdog.hpp"
#include <config.hpp>
#include "serialHandler.hpp"
#include "transceiver.h"
#include "IMU.h"
#include "controller.hpp"
#include "helpers.hpp"

Watchdog watchdog;
Serial pc(USBTX,USBRX);
SerialHandler serial;
DigitalOut led(LED1), led2(LED2), led3(LED3), led4(LED4), radioPower(p30);
AnalogIn battery(p19);
dataStruct data;
configStruct config;
Transceiver radio(p5, p6, p7, p8, p9);
InterruptIn radioInterrupt(p10);
Ticker controllerInterrupt, gyroInterrupt, ledTicker, angleInterrupt, batteryTicker;
Controller controller(p23, p24, p25, p22);
IMU imu;
LocalFileSystem local("local");

uint8_t status;

void batteryLevelUpdate(void){
    data.batteryLevel.f = battery.read()*3.3*(81.6+476)/81.6;
}

void flight(void)
{   
    watchdog.kick();
    controller.update();
}

void ledUpdate(void){
    led4 = 1-led4;
}

int main(void)
{
    //Check if watchdog caused reset
    if ((LPC_WDT->WDMOD >> 2) & 1){
        data.armMotor = false;
        controller.update();
        return;
    }

    loadConfig();

    serial.initialize(); 

    radioPower=1;
    wait(0.5);
    status = radio.initialize();

    if (status)
    {
        ledTicker.attach(&ledUpdate, 0.5);
        while (status)
        {
            radioPower=0;
            wait(0.5);
            radioPower=1;
            wait(0.5);
            status = radio.initialize();
        }
        ledTicker.detach();
    }

    radioInterrupt.fall(callback(&radio, &Transceiver::interruptHandler));

    status = imu.initialize();
    if (status)
    {
        led2 = 1;
        return;
    }

    //imu.calibrate();

    ledTicker.attach(&ledUpdate, 0.5);
    batteryTicker.attach(&batteryLevelUpdate, 0.1);

    led4=1;
    data.newPacket = false;
    while(!data.newPacket){
        data.batteryLevel.f = battery.read()*3.3*(81.6+476)/81.6;
    }

    ledTicker.detach();
    led4 = 0;

    controller.initialize();

    controllerInterrupt.attach(&flight, 1.0/config.flightTickerFrequency);
    gyroInterrupt.attach(callback(&imu, &IMU::updateGyro),1.0/config.gyroTickerFrequency);
    angleInterrupt.attach(callback(&imu, &IMU::updateAngles),1.0/config.angleTickerFrequency);

    watchdog.kick(0.1);
}
