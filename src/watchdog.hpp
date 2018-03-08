#include "mbed.h"

class Watchdog{
    public:
        void kick(float s){
            LPC_WDT->WDCLKSEL = 0x1;
            uint32_t clk = SystemCoreClock / 16;
            LPC_WDT->WDTC = s*(float)clk;
            LPC_WDT->WDMOD = 0x3;
            kick();
        }

        void kick(){
            LPC_WDT->WDFEED = 0xAA;
            LPC_WDT->WDFEED = 0x55;
        }
};