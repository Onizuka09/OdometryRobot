#ifndef MOTOR_H_ 
#define MOTOR_H_
#include "GPIO.h"
#include <stdint.h>

#include "navigation.h"

#define MAX_VELOCITY_LIMIT_COEF 0.5f // 0.2f // half the speed  
#define MOTOR_VELOCITY 94.0f // max velocity is 94 cm/s 
#define MAX_VELOCITY (MOTOR_VELOCITY * MAX_VELOCITY_LIMIT_COEF) 
#define MAX_VELOCITY_ACCELERATION 10.0f


#define MOTOR_ANG_VELOCITY  8.7f 
#define MAX_ANG_VELOCITY_LIMIT_COEF 0.5f // 0.2f // half the speed  
#define MAX_ANGULAR_VEL (MOTOR_ANG_VELOCITY * MAX_ANG_VELOCITY_LIMIT_COEF)
#define MAX_ANGULAR_ACCEL 2.5f


#define IN1_MR PIN_3
#define IN2_MR PIN_5

#define IN3_ML PIN_4
#define IN4_ML PIN_1

#define ENA_MR PIN_1
#define ENB_ML PIN_2

#define IN1_MR_port GPIOB
#define IN2_MR_port GPIOB
#define IN3_ML_port GPIOB
#define IN4_ML_port GPIOB

#define ENA_MR_port GPIOC
#define ENB_ML_port GPIOC 





/**
 * @brief initializes motor both left and right 
 * 
 */
 
void init_motors();

/**
 * @brief controlls motors both left and right (speed  & direction) in CM/s  
 * @param cmdML: takes speed of left motor (can be negative) 
 * @param cmdMR: takes speed of right  motor (can be negative) 
 */
void run_motors(Command_type cmd, float cmdML, float cmdMR);

/**
 * @brief  controlls motor PWM values  
 * 
 * @param pwmL 
 * @param pwmR 
 */
void command_motors(int8_t pwmL, int8_t pwmR);
#endif
