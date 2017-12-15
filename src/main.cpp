#include "mbed.h"
#include "config.h"
#include "transceiver.h"
#include "IMU.h"
#include "controller.h"
#include <iostream>
#include <bitset>

DigitalOut led(LED1), led2(LED2), led3(LED3), led4(LED4);
AnalogIn battery(p20);
dataStruct data;
configStruct config;
Transceiver radio(p5, p6, p7, p8, p9, p10);
Ticker ticker;
Controller controller(p21, p22, p23, p24);
IMU imu;

uint8_t status;

void tick(void){
    radio.update();
    // imu.update();
    controller.update();
    data.batteryLevel = battery.read_u16();
    radio.setAcknowledgePayload(0);
}

int main() {
    // wait(5);
    led=0;
    led2=0;
    led3=0;
    led4=0;

    status = radio.initialize(config, &data);
    if (status) {
        led=0;
        led2=1;
        led3=0;
        led4=1;
        return 0;
    }
    led=1;
    // imu.initialize(&data);
    
    led2=1;

    radio.update();

    while (data.remote.throttle > 25){
        radio.update();
        wait_ms(100);
        if (data.remote.missedPackets>200){
            led=1;
            led2=0;
            led3=0;
            led4=1;
        } else {
            led=0;
    led2=0;
    led3=0;
    led4=0;
        }
        // std::cout << data.remote.throttle << std::endl;
    }

    led3=1;
    
    while (data.remote.throttle < 1000){
        radio.update();
        wait_ms(100);
    //     if (data.remote.missedPackets>9){
    //         led=1;
    //         led2=0;
    //         led3=0;
    //         led4=1;
    //     } else {
    //         led=0;
    // led2=0;
    // led3=0;
    // led4=0;
    //     }
    }

    led4=1;

    while (data.remote.throttle > 25){
        radio.update();
        wait_ms(100);
        if (data.remote.missedPackets>9){
            led=1;
            led2=0;
            led3=0;
            led4=1;
        } else {
            led=0;
    led2=0;
    led3=0;
    led4=0;
        }
    }

    led=0;
    led2=0;
    led3=0;
    led4=0;
    controller.initialize(&data, &config.controllerConfig);

    ticker.attach(&tick,config.tickerPeriod);
}
