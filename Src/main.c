#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "encoder.h"
#include "delay.h"
#include "motor.h"
#include "pwm.h"
#include "bsp.h"
#include "Debug_dirver.h"
#include "odometry.h"
#include "system_clk.h"
#include "odometry.h"
#include "navigation.h"
#include "test.h"

volatile uint32_t odo_time = 0, curr_odo_time = 0, prev_odo_time = 0;
OdometryTypedef odo = {0};

void test_max_velocity();
void test_max_ang_velocity();

int main(void)
{
    // Initialize GPIOA, GPIOB, and GPIOC

    systemClock_Config();

    GPIO_init();
    Led_init();
    InitSystick();
    debug_uart_init();

    GPIO_Config_Output(GPIOA, PIN_5);
    GPIO_PIN_WRITE(GPIOA, PIN_5, 0);
    btn_init();
    // PC13_EXTI_Config();
    init_motors();
    // Configure encoder interrupts
    GPIO_LeftEncoder_config();
    GPIO_RightEncoder_config();
    timer1_LeftEncoder_confifg();
    timer3_RightEncoder_confifg();
    OdoInit(&odo);
    // OdoUpdate(&odo,10);
    uint8_t btn_state = 0;
    while (1)
    {
        // test_encoder(); 
        btn_state = read_btn_status();
        if (btn_state)
        {
            // test_max_ang_velocity();
            // command_motors(MIN_PWM_SPEED, MIN_PWM_SPEED);
            // move_angular_speed(5, &odo);
            position_control(100, &odo);
            delay_ms(10);     
            angle_control(DEG2RAD(180),&odo);
            delay_ms(10);     
            position_control(100, &odo);
            delay_ms(10);     

            btn_state = 0;
        }
        // print_console("Hello world");
        
    }

    return 0;
}

void test_max_velocity()
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
        OdoUpdate(&odo, dt);
        command_motors(MAX_PWM_SPEED, MAX_PWM_SPEED);

        delay_ms(10);
    }
    command_motors(0, 0);
}
void test_max_ang_velocity()
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
        OdoUpdate(&odo, dt);
        command_motors(-MAX_PWM_SPEED, MAX_PWM_SPEED); // w=8.8
        // command_motors(MAX_PWM_SPEED, -MAX_PWM_SPEED); // w=8.6

        delay_ms(10);
    }
    command_motors(0, 0);
}