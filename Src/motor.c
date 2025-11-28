#include "motor.h"
#include "pwm.h"



void init_motors()
{

    GPIO_Config_Output(IN4_ML_port, IN4_ML);
    GPIO_Config_Output(IN3_ML_port, IN3_ML);
    GPIO_Config_Output(IN2_MR_port, IN2_MR);
    GPIO_Config_Output(IN1_MR_port, IN1_MR);

    PWM_TIM15_CH1_Config();
    PWM_TIM15_CH2_Config();
    PWM_TIM15_CH1_Start();
    PWM_TIM15_CH1_SetDutyCyle(0);
    PWM_TIM15_CH2_SetDutyCyle(0);
}


void run_motors(int8_t speedML, int8_t speedMR)
{
    GPIO_PIN_WRITE(GPIOB, IN3_ML, 0);
    GPIO_PIN_WRITE(GPIOB, IN4_ML, 1);

    GPIO_PIN_WRITE(GPIOB, IN1_MR, 1);
    GPIO_PIN_WRITE(GPIOB, IN2_MR, 0);

    PWM_TIM15_CH1_SetDutyCyle(speedML);
    PWM_TIM15_CH2_SetDutyCyle(speedMR);
}

