#include "ESCController.hpp"

#define max(a, b) ({ __typeof__ (a) _a = (a);  __typeof__ (b) _b = (b);  _a > _b ? _a : _b; })
#define min(a, b) ({ __typeof__ (a) _a = (a);  __typeof__ (b) _b = (b);  _a < _b ? _a : _b; })

void ESCController::initialize(void)
{
    pwm.enableOneshot();
    pwm.period_us(1000);
    pwm.pulsewidth_us(125); // 125 us idle
    pwm.fireOneShot();
}

void ESCController::update(float value)
{
    pwm.pulsewidth_us(min(max(value + 125.0, 125.0), 250.0));
}

void ESCController::fire(void)
{
    pwm.fireOneShot();
}
