#ifndef MOTOR_H_ 
#define MOTOR_H_
#include "timer.h"
#include "GPIO.h"
#include <stdint.h>
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


typedef enum
{
    Forward,
    Backward, // Fixed typo: "Backword" to "Backward"
    Left,
    Right
} Directions;
void init_motor(int PSC,int  ARR);
void forward(uint8_t speedM1, uint8_t speedM2);
void backward(uint8_t speedM1, uint8_t speedM2);
void left(uint8_t speedM1, uint8_t speedM2) ;
void right(uint8_t speedM1, uint8_t speedM2);
void stop();
#endif
