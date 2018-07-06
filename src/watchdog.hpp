#include "mbed.h"

class Watchdog{
    private:
        static Callback<void()> watchdogCallback;
    public:
        static void kick(float s){
            LPC_WDT->WDCLKSEL = 0x1;
            uint32_t clk = SystemCoreClock / 16;
            LPC_WDT->WDTC = s*(float)clk;
            NVIC_SetVector(WDT_IRQn, (uint32_t)Watchdog::irq_handler);
            LPC_WDT->WDMOD = 0x3;
            kick();
        }

        static void kick(){
            LPC_WDT->WDFEED = 0xAA;
            LPC_WDT->WDFEED = 0x55;
        }

        static void irq_handler(void){
            LPC_WDT->WDMOD = 1<<3;
            //callback.call();
        }

        static void attach(Callback<void()> function){
            watchdogCallback = function;
        }
};

Callback<void()> Watchdog::watchdogCallback;