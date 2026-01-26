#ifndef TEST_H 
#define TEST_H
#include "motor.h" 

#include <main.h>
void test_pwm() ; 

int test_encoder() ;

void test_max_velocity(OdometryTypedef* odo);

void test_max_ang_velocity(OdometryTypedef* odo);
void test_uart(); 
#endif 