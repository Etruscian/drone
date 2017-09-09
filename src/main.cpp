#include "mbed.h"
#include "controller.h"
#include "transceiver.h"
#include "config.h"
#include <iostream>
#include <bitset>

DigitalOut led(LED1), led2(LED2), led3(LED3);
AnalogIn battery(p20);
dataStruct data;
configStruct config;
Transceiver radio(p5, p6, p7, p8, p9, p10);
Ticker ticker;
Controller controller(p21, p22, p23, p24);

uint8_t status;

 void tick(void){
    radio.update(&data);
    controller.update(&data);
    data.batteryLevel = battery.read_u16();
    // std::cout<<data.batteryLevel<<endl;
    radio.setAcknowledgePayload(0,&data);
}

int main() {
    status = radio.initialize(config.channel,config.rxAddress, config.txAddress,config.transferSize);
    if (status) {
        led2 = 1;
        return 0;
    }

    led=0;

    while (!data.armMotor){
        radio.update(&data);
    }

    led=1;

    controller.initialize();

    ticker.attach(&tick,config.tickerPeriod);
}
