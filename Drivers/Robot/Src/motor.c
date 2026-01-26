#include "motor.h"
#include "pwm.h"
#include <math.h>
#include <main.h>
#include <stm32g070xx.h>
float pwmMl = 0;
float pwmMr = 0;
volatile int8_t pwml_dbg = 0;
volatile int8_t pwmr_dbg = 0;
void init_motors()
{

    // GPIO_Config_Output(IN4_ML_port, IN4_ML);
    // GPIO_Config_Output(IN3_ML_port, IN3_ML);
    // GPIO_Config_Output(IN2_MR_port, IN2_MR);
    // GPIO_Config_Output(IN1_MR_port, IN1_MR);

    // PWM_TIM15_CH1_Config();
    // PWM_TIM15_CH2_Config();
    PWM_TIM15_Start();

    PWM_TIM15_CH1_SetDutyCyle(0);
    PWM_TIM15_CH2_SetDutyCyle(0);
}

void run_motors(Command_type cmd, float cmdML, float cmdMR)
{
    if (cmd == POSITION_CMD)
    {
        // convert cmd from cm/s to pwm value
        pwmMl = (cmdML / MAX_VELOCITY) * MAX_PWM_SPEED;
        pwmMr = (cmdMR / MAX_VELOCITY) * MAX_PWM_SPEED;
        // If the command is positive, ensure it is at least MIN_PWM
        // if (pwmMl > 0.1f)  pwmMl += MIN_PWM_SPEED ;
        // if (pwmMl < -0.1f) pwmMl -= MIN_PWM_SPEED;

        // if (pwmMr > 0.1f)  pwmMr += MIN_PWM_SPEED ;
        // if (pwmMr < -0.1f) pwmMr -= MIN_PWM_SPEED;
    }
    else if (cmd == ANGULAR_CMD)
    {
        pwmMl = (cmdML / MAX_ANGULAR_VEL) * MAX_PWM_SPEED;
        pwmMr = (cmdMR / MAX_ANGULAR_VEL) * MAX_PWM_SPEED;
    }
    else
    {
        __NOP();
    }
    // clamping
    if (pwmMl > MAX_PWM_SPEED)
        pwmMl = MAX_PWM_SPEED;
    if (pwmMl < -MAX_PWM_SPEED)
        pwmMl = -MAX_PWM_SPEED;

    if (pwmMr > MAX_PWM_SPEED)
        pwmMr = MAX_PWM_SPEED;
    if (pwmMr < -MAX_PWM_SPEED)
        pwmMr = -MAX_PWM_SPEED;

    command_motors(pwmMl, pwmMr);
}
void command_motors(int8_t pwmL, int8_t pwmR)
{

    if (pwmL >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, IN3_ML, 0);
        HAL_GPIO_WritePin(GPIOB, IN4_ML, 1);
    }
    else
    { // motor bech iwa5er
        HAL_GPIO_WritePin(GPIOB, IN3_ML, 1);
        HAL_GPIO_WritePin(GPIOB, IN4_ML, 0);
        pwmL = pwmL * -1;
    }

    if (pwmR >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, IN1_MR, 1);
        HAL_GPIO_WritePin(GPIOB, IN2_MR, 0);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, IN1_MR, 0);
        HAL_GPIO_WritePin(GPIOB, IN2_MR, 1);
        pwmR = pwmR * -1;
    }
    pwml_dbg = pwmL;
    pwmr_dbg = pwmR;
    // el compiler yetmanyek aliya mahbech ye9blha fuck you gcc
    // pwmL=abs(pwmL);
    // pwmR=abs(pwmR);

    PWM_TIM15_CH2_SetDutyCyle(pwmR);
    PWM_TIM15_CH1_SetDutyCyle(pwmL);
}
