#include "delay.h"
#include "motor.h"
#include "Debug_dirver.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
volatile unsigned long SystickCnt = 0;

#define sensor_arr_size 5

// Wheel radius and distance calculations converted to integer-based (in millimeters)
// wheelRadius = 6.7 / 2 = 3.35 cm = 33.5 mm
#define WHEEL_RADIUS_MM 335      // 3.35 cm * 100 = 335 mm (scaled by 10 for precision)
#define PULSE_PER_REVOLUTION 442 // 34 * 13
// distancePerPulse = (pi * 6.7) / (34 * 13) ≈ 0.0477 cm/pulse = 0.477 mm/pulse
// To avoid floating-point, scale by 1000: 477 mm/1000 per pulse
#define DISTANCE_PER_PULSE_NUM 477  // Numerator for distance per pulse (in mm * 1000)
#define DISTANCE_PER_PULSE_DEN 1000 // Denominator for scaling

// Pin configurations remain unchanged
// IN1 PB5  (D4)  // output
// IN2 PB4  (D5)   // output
// IN3 PB1  (x)  infront of D7 // output
// IN4 PB3  (D3)  // output
// ENA PA0 : PWM_timer2_ch1 (A0)
// ENB PA1 : PWM_timer2_ch2 (A1)
// TCRT5000 sensor pins (5 pins INPUT)
// PIN_1 PB12 => S1 left
// PIN_2 PB2  => S2
// PIN_3 PB15 => S3 middle
// PIN_4 PB14 => S4
// PIN_5 PA13 => S5 right

#define DESIRED_DISTANCE_MM 400 // 15 cm = 150 mm

typedef enum
{
    Forward,
    Backward, // Fixed typo: "Backword" to "Backward"
    Left,
    Right
} Directions;

void PC13_EXTI_Config();

void test_motor(Directions d, int speed);
uint16_t cntA = 0;
uint16_t cntB = 0;
char buff[255] = {0};
volatile uint32_t average_distance_mm = 0; // Changed to integer (in mm)
int tmpCntA = 0, tmpCntB = 0;
int prevCntA = 0, prevCntB = 0;

int main(void)
{
    // Initialize GPIOA, GPIOB, and GPIOC
    GPIO_init();
    InitSystick();
    // debug_uart_init();

    GPIO_Config_Output(GPIOA, PIN_5);
    GPIO_PIN_WRITE(GPIOA, PIN_5, 1);
    GPIO_Config_Input(GPIOC, PIN_13); // Configure internal button as input
    PC13_EXTI_Config();
    // Configure encoder interrupts
    // GPIO_LeftEncoder_config();
    // GPIO_RightEncoder_config();
    // timer1_LeftEncoder_confifg();
    // timer3_RightEncoder_confifg();
    init_motor();

    while (1)
    {
        for (int d = 0; d <= 99; d++)
        {
            TIM15->CCR1 = d;
            delay_ms(10); // Adjust speed of fade
        }

        delay_ms(500);

        // Fade out (100% to 0%)
        for (int d = 99; d > 0; d--)
        {
            TIM15->CCR1 = d;
            delay_ms(10); // Adjust speed of fade
        }

        delay_ms(500);
        // forward(50, 50);
        // GPIOA->ODR &= ~(1U << PIN_5); // turn on LED
        // delay_ms(1000);
        // GPIOA->ODR |= (1U << PIN_5); // turn on LED
        // delay_ms(1000);

        // Print current average distance
        // snprintf(buff, sizeof(buff), "desired dist %lu mm\n\r", average_distance_mm);
        // print_console(buff);
        // memset(buff, 0, sizeof(buff));

        // forward(100, 100);

        // Loop until the desired distance is reached
        // while (average_distance_mm <= DESIRED_DISTANCE_MM)
        // {
        // 	GPIOA->ODR |= (1U << PIN_5); // turn on LED
        //     // snprintf(buff, sizeof(buff), "desired dist %lu mm\n\r", average_distance_mm);
        //     // print_console(buff);
        //     // memset(buff, 0, sizeof(buff));
        //     forward(20, 20);

        //     // Read encoder counts
        //     cntA = TIM1->CNT >> 2;
        //     cntB = TIM3->CNT >> 2;

        //     // Calculate distance in mm for each wheel
        //     // dist = (cnt * DISTANCE_PER_PULSE_NUM) / DISTANCE_PER_PULSE_DEN
        //     uint32_t distA_mm = (cntA * DISTANCE_PER_PULSE_NUM) / DISTANCE_PER_PULSE_DEN;
        //     uint32_t distB_mm = (cntB * DISTANCE_PER_PULSE_NUM) / DISTANCE_PER_PULSE_DEN;

        //     // Average distance in mm
        //     average_distance_mm = (distA_mm + distB_mm) / 2;
        // 	// delay_ms(10);

        // }
        // stop();
    }

    return 0;
}

void test_motor(Directions d, int speed)
{
    int x = speed;
    switch (d)
    {
    case Forward:
        forward(x, x);
        break;
    case Backward:
        backward(x, x);
        break;
    case Left:
        left(x, x);
        break;
    case Right:
        right(x, x);
        break;
    default:
        break;
    }
    delay_ms(5000);
    stop();
}

void PC13_EXTI_Config()
{
    __disable_irq();
    // Enable clock access to SYSCFG
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
    // Connect EXTI line with PC13

    // SYSCFG->EXTICR[3] &= ~(0xf << 4); // Reset
    // SYSCFG->EXTICR[3] |= (0x2 << 4);  // Connect EXTI line EXTI 15_10
    // // Unmask EXTI13
    // EXTI->IMR |= EXTI_IMR_IM13;
    // // Select rising edge trigger
    // EXTI->RTSR |= EXTI_RTSR_TR13;
    // EXTI->FTSR &= ~EXTI_FTSR_TR13;
    // // Enable EXTI13 line in NVIC
    // NVIC_SetPriority(EXTI15_10_IRQn, 1);
    // NVIC_EnableIRQ(EXTI15_10_IRQn);
    __enable_irq();
}

// EXTI15_10_IRQHandler remains commented out as in the original
/*
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << 13))
    {
        EXTI->PR |= (1U << 13); // Clear the pending bit
        GPIOA->ODR ^= (1U << PIN_5);
    }
    return;
}
*/
void SysTick_Handler(void)
{
    SystickCnt++;
}