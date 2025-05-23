#include "motor.h"

void init_motor()
{

    GPIO_Config_Output(IN4_ML_port, IN4_ML);
    GPIO_Config_Output(IN3_ML_port, IN3_ML);
    GPIO_Config_Output(IN2_MR_port, IN2_MR);
    GPIO_Config_Output(IN1_MR_port, IN1_MR);

    GPIO_PWM_LeftMotor_config();
    GPIO_PWM_RightMotor_config();
    Timer15_init();
    Timer15_PWM_channel1_config();
    Timer15_PWM_channel2_config();
    Timer15_Enable();
}

void backward(uint8_t speedM1, uint8_t speedM2)
{
    GPIO_PIN_WRITE(GPIOB, IN3_ML, 0);
    GPIO_PIN_WRITE(GPIOB, IN4_ML, 1);

    GPIO_PIN_WRITE(GPIOB, IN1_MR, 0);
    GPIO_PIN_WRITE(GPIOB, IN2_MR, 1);

    Timer15_set_dutyCycle_ch1(speedM1);
    Timer15_set_dutyCycle_ch2(speedM2);
}
void forward(uint8_t speedM1, uint8_t speedM2)
{
    GPIO_PIN_WRITE(GPIOB, IN3_ML, 1);
    GPIO_PIN_WRITE(GPIOB, IN4_ML, 0);

    GPIO_PIN_WRITE(GPIOB, IN1_MR, 1);
    GPIO_PIN_WRITE(GPIOB, IN2_MR, 0);

    Timer15_set_dutyCycle_ch1(speedM1);
    Timer15_set_dutyCycle_ch2(speedM2);
}
void right(uint8_t speedM1, uint8_t speedM2)
{

    GPIO_PIN_WRITE(GPIOB, IN3_ML, 0);
    GPIO_PIN_WRITE(GPIOB, IN4_ML, 1);

    GPIO_PIN_WRITE(GPIOB, IN1_MR, 1);
    GPIO_PIN_WRITE(GPIOB, IN2_MR, 0);
    Timer15_set_dutyCycle_ch1(speedM1);
    Timer15_set_dutyCycle_ch2(speedM2);
}
void left(uint8_t speedM1, uint8_t speedM2)
{
    GPIO_PIN_WRITE(GPIOB, IN3_ML, 1);
    GPIO_PIN_WRITE(GPIOB, IN4_ML, 0);

    GPIO_PIN_WRITE(GPIOB, IN1_MR, 1);
    GPIO_PIN_WRITE(GPIOB, IN2_MR, 0);

    Timer15_set_dutyCycle_ch1(speedM1);
    Timer15_set_dutyCycle_ch2(speedM2);
}
void stop()
{

    GPIO_PIN_WRITE(GPIOB, IN3_ML, 0);
    GPIO_PIN_WRITE(GPIOB, IN4_ML, 0);

    GPIO_PIN_WRITE(GPIOB, IN1_MR, 0);
    GPIO_PIN_WRITE(GPIOB, IN2_MR, 0);
    Timer15_set_dutyCycle_ch1(0);
    Timer15_set_dutyCycle_ch2(0);
}
