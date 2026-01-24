#ifndef PWM_H_ 
#define PWM_H_ 
#define MAX_PWM_SPEED 100 
#define MIN_PWM_SPEED 10

void PWM_TIM15_Start(); 
void PWM_TIM15_CH1_SetDutyCyle(uint8_t t); 
void PWM_TIM15_CH2_SetDutyCyle(uint8_t t); 





#endif 
