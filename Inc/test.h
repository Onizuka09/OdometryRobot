#ifndef TEST_H 
#define TEST_H
#include "motor.h" 

void pwm_test() ; 
int test_encoder(TIM_TypeDef* tim) ;
void test_motor(Directions d, int speed) ; 
#endif 