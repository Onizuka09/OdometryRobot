#include "test.h"
#include "delay.h"
void pwm_test()
{
    for (int d = 0; d <= 99; d++)
    {
        TIM15->CCR1 = d;
        delay_ms(10); // Adjust speed of fade
    }

    delay_ms(500);

    // Fade out (100% to 0%)
    for (int d = 99; d > 0; d--)
    {
        TIM15->CCR1 = d;
        delay_ms(10); // Adjust speed of fade
    }
}
int test_encoder(TIM_TypeDef* tim)
{
    return tim->CNT;
}
void test_motor(Directions d, int speed)
{
    int x = speed;
    switch (d)
    {
    case Forward:
        forward(x, x);
        break;
    case Backward:
        backward(x, x);
        break;
    case Left:
        left(x, x);
        break;
    case Right:
        right(x, x);
        break;
    default:
        break;
    }
    delay_ms(5000);
    stop();
}