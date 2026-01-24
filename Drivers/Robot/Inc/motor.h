#ifndef MOTOR_H_ 
#define MOTOR_H_
#include <stdint.h>
#include <main.h> 
#include "navigation.h"
#include "pwm.h"
#define MAX_VELOCITY_LIMIT_COEF 0.5f // 0.2f // half the speed  
#define MOTOR_VELOCITY 94.0f // max velocity is 94 cm/s 
#define MAX_VELOCITY (MOTOR_VELOCITY * MAX_VELOCITY_LIMIT_COEF) 
#define MAX_VELOCITY_ACCELERATION 10.0f


#define MOTOR_ANG_VELOCITY  8.7f 
#define MAX_ANG_VELOCITY_LIMIT_COEF 0.5f // 0.2f // half the speed  
#define MAX_ANGULAR_VEL (MOTOR_ANG_VELOCITY * MAX_ANG_VELOCITY_LIMIT_COEF)
#define MAX_ANGULAR_ACCEL 2.5f


#define IN1_MR BIN1_Pin
#define IN2_MR BIN2_Pin

#define IN3_ML AIN1_Pin
#define IN4_ML AIN2_Pin

#define ENA_MR PWMB_Pin
#define ENB_ML PWMA_Pin



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
