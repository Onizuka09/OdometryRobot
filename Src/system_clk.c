#include "system_clk.h"
#include <stm32g070xx.h>

void systemClock_Config()
{
    // enabling HSI 
    RCC->CR |= RCC_CR_HSION;
    // waiting the HSI to be ready 
    while (!(RCC->CR & RCC_CR_HSIRDY)) ;
    // setting clock devider to produce HSISYS to 1 
    RCC->CR &= ~(0UL << 11) ;
    // enable the power controller 
    RCC->APBENR1 |= RCC_APBENR1_PWREN;

    // Voltage Regulator for range 2 (VOS2) sufficient for 16 MHz
    PWR->CR1 |= PWR_CR1_VOS_1 ; 
    
    // Wait for regulator to be ready
    while ((PWR->SR2 & PWR_SR2_VOSF)) ;
    
    // Flash latency for 16 MHz (0 wait states)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_0; // 0 wait states for ≤24 MHz @ 2.0V
    
    // 7. Configure AHB, APB prescalers
    RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1 (16 MHz)
    RCC->CFGR &= ~RCC_CFGR_PPRE; // APB prescaler = 1 (16 MHz)
    
    // Select HSI as system clock source
    RCC->CFGR |= RCC_CFGR_SWS_HSISYS; // Select HSI as system clock
    
    // Wait until HSI is used as system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSISYS) ;
    
    //Disable PLL and other clock sources to save power
    RCC->CR &= ~RCC_CR_PLLON;  // Ensure PLL is off
}