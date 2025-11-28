#include "test.h"
#include "delay.h"
void pwm_test()
{
    for (int d = 0; d <= 99; d++)
    {
        TIM15->CCR1 = d;
        TIM15->CCR2 = d;

        delay_ms(10); // Adjust speed of fade
    }

    delay_ms(500);

    // Fade out (100% to 0%)
    for (int d = 99; d > 0; d--)
    {
        TIM15->CCR1 = d;
        TIM15->CCR2 = d;

        delay_ms(10); // Adjust speed of fade
    }
}
int test_encoder(TIM_TypeDef* tim)
{
    return tim->CNT;
}
