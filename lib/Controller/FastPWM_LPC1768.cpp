#ifdef TARGET_LPC176X

#include "FastPWM.h"

void FastPWM::initFastPWM( void ) {
    //Set clock source
    LPC_SC->PCLKSEL0|=1<<12;
    LPC_PWM1->PCR &= 0 << _pwm.pwm;
    bits = 32;
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    *(_pwm.MR) = ticks;
    LPC_PWM1->LER |= 1 << _pwm.pwm;
}

void FastPWM::period_ticks( uint32_t ticks ) {
    LPC_PWM1->MR0 = ticks;
    LPC_PWM1->LER |= 1 << 0;
}

uint32_t FastPWM::getPeriod( void ) {
    return LPC_PWM1->MR0;
}

//Maybe implemented later, but needing to change the prescaler for a 32-bit
//timer used in PWM mode is kinda unlikely.
//If you really need to do it, rejoice, you can make it run so slow a period is over 40,000 year
uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
    //Disable dynamic prescaling
    dynamicPrescaler = false;
    
    return 1;
}

void FastPWM::enableOneshot(void){
    LPC_PWM1->MCR &= 0b101 << _pwm.pwm;
    this->oneshotEnabled = true;
}

void FastPWM::disableOneshot(void){
    LPC_PWM1->MCR |= 0b010 << _pwm.pwm;
}

void FastPWM::fireOneShot(void){
    if (this->oneshotEnabled)
        LPC_PWM1->TCR |= 0b1011;
}
#endif