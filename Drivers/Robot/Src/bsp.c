#include "bsp.h"
#include <stdint.h>
#include <main.h>


void Toggle_led()
{

    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
}
void TurOn_led()
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET);
}
void TurOff_led()
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);
}
uint8_t read_btn_status()
{
    if (HAL_GPIO_ReadPin(BTN_GPIO_Port,BTN_Pin))
    {
        return 0;
    }
    else
    {

        TurOn_led();
        return 1;
    }
    return 0;
}
