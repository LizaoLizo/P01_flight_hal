#include "Int_IP5305T.h"

void Int_IP5305T_start(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      GPIO_PIN_RESET);

    vTaskDelay(pdMS_TO_TICKS(100));

    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      GPIO_PIN_SET);
}

void Int_IP5305T_shutdown(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      GPIO_PIN_RESET);

    vTaskDelay(pdMS_TO_TICKS(100));

    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      GPIO_PIN_SET);

    vTaskDelay(200);

        HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      GPIO_PIN_RESET);

    vTaskDelay(pdMS_TO_TICKS(100));

    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      GPIO_PIN_SET);

}