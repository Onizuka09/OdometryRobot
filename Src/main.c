#include "delay.h"
#include "motor.h"
#include "pwm.h"
#include "Debug_dirver.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "test.h"
volatile unsigned long SystickCnt = 0;

#define sensor_arr_size 5

// Wheel radius and distance calculations converted to integer-based (in millimeters)
#define WHEELBASE_MM 220 // 15cm between wheels
// wheelRadius = 6.7 / 2 = 3.35 cm = 33.5 mm
#define WHEEL_RADIUS_MM 335      // 3.35 cm * 100 = 335 mm (scaled by 10 for precision)
#define PULSE_PER_REVOLUTION 442 // 34 * 13
// distancePerPulse = (pi * 6.7) / (34 * 13) ≈ 0.0477 cm/pulse = 0.477 mm/pulse
// To avoid floating-point, scale by 1000: 477 mm/1000 per pulse
#define DISTANCE_PER_PULSE_NUM 477  // Numerator for distance per pulse (in mm * 1000)
#define DISTANCE_PER_PULSE_DEN 1000 // Denominator for scaling

#define MM_PER_COUNT (2 * 31415 * WHEEL_RADIUS_MM / PULSE_PER_REVOLUTION) // 2πr/cpr

#define DESIRED_DISTANCE_MM 400     // 15 cm = 150 mm
#define DESIRED_TURN_DEG 720 * 2    // 15 cm = 150 mm
#define RAD_TO_DEG (180000 / 31415) // 180/π × 1000
void PC13_EXTI_Config();

void test_motor(Directions d, int speed);
// uint16_t cntA = 0;
// uint16_t cntB = 0;
// int16_t cntADeg = 0;
// int16_t cntBDeg = 0;

char buff[255] = {0};


volatile bool btn = false;

bool btn_clk = false;
void prepare_movement() {
    TIM3->CNT = 0;
    TIM1->CNT = 0;
    delay_ms(10);  // Allow time for reset
}
void moveRobot()
{
    prepare_movement(); 
    uint32_t average_distance_mm = 0;
    uint16_t cntA = 0;
    uint16_t cntB = 0;
    while (average_distance_mm <= DESIRED_DISTANCE_MM)
    {
        GPIOA->ODR |= (1U << PIN_5); // turn on LED
        // snprintf(buff, sizeof(buff), "desired dist %lu mm\n\r", average_distance_mm);
        // print_console(buff);
        // memset(buff, 0, sizeof(buff));
        forward(60, 60);

        // Read encoder counts
        cntA = TIM1->CNT >> 2;
        cntB = TIM3->CNT >> 2;

        // Calculate distance in mm for each wheel
        // dist = (cnt * DISTANCE_PER_PULSE_NUM) / DISTANCE_PER_PULSE_DEN
        uint32_t distA_mm = (cntA * DISTANCE_PER_PULSE_NUM) / DISTANCE_PER_PULSE_DEN;
        uint32_t distB_mm = (cntB * DISTANCE_PER_PULSE_NUM) / DISTANCE_PER_PULSE_DEN;

        // Average distance in mm
        average_distance_mm = (distA_mm + distB_mm) / 2;
        // delay_ms(10);
    }
    stop();
    delay_ms(1000);
}
void turnRobot()
{
    prepare_movement(); 
    uint32_t average_turn_deg = 0; // Changed to integer (in mm)
    int16_t cntADeg = 0;
    int16_t cntBDeg = 0;
    int32_t delta = 0;
    while (abs(average_turn_deg) <= (DESIRED_TURN_DEG))
    {
        GPIOA->ODR |= (1U << PIN_5); // turn on LED
                                     // left motor Direction 1 (+)
        TIM3->CNT = 0;
        TIM1->CNT = 0;
        GPIO_PIN_WRITE(GPIOB, IN3_ML, 0);
        GPIO_PIN_WRITE(GPIOB, IN4_ML, 1);

        // right motor direction 2  (-)
        GPIO_PIN_WRITE(GPIOB, IN1_MR, 0);
        GPIO_PIN_WRITE(GPIOB, IN2_MR, 1);

        // speed
        PWM_TIM15_CH1_SetDutyCyle(60);
        PWM_TIM15_CH2_SetDutyCyle(60);
        // forward(60, 60);

        // Read encoder counts
        cntADeg = (int16_t)TIM1->CNT >> 2; // right
        cntBDeg = (int16_t)TIM3->CNT >> 2; // left

        // delta = (cntBDeg - cntADeg) * 814; //  (360000  / PULSE_PER_REVOLUTION );
        delta =
            (cntBDeg - cntADeg) // Difference in counts
            * MM_PER_COUNT      // Convert to mm traveled difference
            * RAD_TO_DEG        // Convert to degrees
            / WHEELBASE_MM;     // Normalize by wheelbase
        average_turn_deg += delta / 1000;
    }
    stop();
}
int main(void)
{
    // Initialize GPIOA, GPIOB, and GPIOC

    GPIO_init();
    InitSystick();
    // debug_uart_init();

    GPIO_Config_Output(GPIOA, PIN_5);
    GPIO_PIN_WRITE(GPIOA, PIN_5, 0);
    GPIO_Config_Input(GPIOC, PIN_13); // Configure internal button as input
    PC13_EXTI_Config();
    init_motor(15, 99);
    // Configure encoder interrupts
    GPIO_LeftEncoder_config();
    GPIO_RightEncoder_config();
    timer1_LeftEncoder_confifg();
    timer3_RightEncoder_confifg();

    while (1)
    {
        if (btn)
        {
            btn = false;

            btn_clk = true;
        }
        if (btn_clk)
        {
            // forward 40 CM
            moveRobot();
            TIM3->CNT = 0;
            TIM1->CNT = 0;
            delay_ms(1000);
            GPIOA->ODR &= ~(1U << PIN_5); // turn on LED
            // turn 180 Deg
            turnRobot();
            TIM3->CNT = 0;
            TIM1->CNT = 0;
            GPIOA->ODR &= ~(1U << PIN_5); // turn on LED
            delay_ms(1000);
            // move Robot 40 CM 
            moveRobot();
            TIM3->CNT = 0;
            TIM1->CNT = 0;
            delay_ms(1000);
            GPIOA->ODR &= ~(1U << PIN_5); // turn on LED
            btn_clk = false;
        }

    }

    return 0;
}

void PC13_EXTI_Config()
{
    __disable_irq();
    // Enable clock access to SYSCFG
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
    // RCC->APBENR2 |= RCC_AHBENR_EX;

    // Connect EXTI line with PC13
    // x = 13 / 4 = 3.25 => get rid of the deciaml number => 3
    // use the formula m = 4*2 = 8 => m+3 = 11
    EXTI->EXTICR[3] |= (0x2 << 8); // PC13 (0x02)

    // Unmask EXTI13
    EXTI->IMR1 |= EXTI_IMR1_IM13;
    // Select rising edge trigger
    EXTI->RTSR1 |= EXTI_RTSR1_RT13;
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT13;
    // Enable EXTI13 line in NVIC
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    __enable_irq();
}

// EXTI4_15_IRQn

void EXTI4_15_IRQHandler(void)
{
    if (EXTI->RPR1 & (1U << 13))
    {
        EXTI->RPR1 |= (1U << 13); // Clear the pending bit
        GPIOA->ODR ^= (1U << PIN_5);
        btn = true;
    }
    return;
}

void SysTick_Handler(void)
{
    SystickCnt++;
}