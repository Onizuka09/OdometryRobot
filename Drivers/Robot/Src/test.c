#include "test.h"
#include "delay.h"
#include <stm32g070xx.h>
#include "debug_driver.h"
#include "motor.h" 
#include "encoder.h"
#include <stdint.h>
volatile  uint16_t  encL = 0 ; 
volatile uint16_t encR = 0 ; 
void pwm_test()
{
    for (int8_t i = 0; i < MAX_PWM_SPEED; i += 10)
    {
        command_motors(i, i);
        HAL_Delay(50);
    }

    command_motors(0, 0);
    for (int8_t i = MAX_PWM_SPEED; i > 0  ; i -= 10)
    {
        command_motors(i, i);
        HAL_Delay(50);
    }
    command_motors(0, 0);
}
int test_encoder( )
{
    encL = timer1_LeftEncoder_Read(); 
    encR = timer3_RightEncoder_Read(); 
    return 0 ;
}

void test_uart(){ 
    printf("hellow World\n\r"); 
    print_console("hello form debug console");
    
}
void test_max_velocity(OdometryTypedef* odo)
{
    float last = 0.0f, now = 0.0f;
    float dt = 0.0;
    while (1)
    {
        now = get_current_time_ms();
        dt = (float)(now - last) / 1000;
        if (dt <= 0)
            dt = 0.001f;
        last = now;
        OdoUpdate(odo, dt);
        command_motors(MAX_PWM_SPEED, MAX_PWM_SPEED);

        delay_ms(10);
    }
    command_motors(0, 0);
}
void test_max_ang_velocity(OdometryTypedef* odo)
{
    float last = 0.0f, now = 0.0f;
    float dt = 0.0;
    while (1)
    {
        now = get_current_time_ms();
        dt = (float)(now - last) / 1000;
        if (dt <= 0)
            dt = 0.001f;
        last = now;
        OdoUpdate(odo, dt);
        command_motors(-MAX_PWM_SPEED, MAX_PWM_SPEED); // w=8.8
        // command_motors(MAX_PWM_SPEED, -MAX_PWM_SPEED); // w=8.6

        delay_ms(10);
    }
    command_motors(0, 0);
}