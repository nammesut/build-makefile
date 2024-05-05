/*
 * LED_Toggle.c
 *
 *  Created on: Apr 11, 2024
 *      Author: DELL
 */
#include "stm32f103xx_gpio_driver.h"

void delay()
{
	for(uint32_t i = 0; i < 500000; i++)
	{

	}
}

int main()
{
	GPIO_Handle_t GPIO_Led, GPIO_BTN;

	GPIO_Led.pGPIOx = GPIOC;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Led.GPIO_PinConfig.GPIO_PinType = GPIO_MODE_OUT_HIGH_SPEED;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;

	GPIO_PeriCockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Led);

	// GPIO_Led.pGPIOx = GPIOB;
	// GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	// GPIO_Led.GPIO_PinConfig.GPIO_PinType = GPIO_MODE_IN;
	// GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PLPD;

	// GPIO_PeriCockControl(GPIOB, ENABLE);
	// GPIO_Init(&GPIO_BTN);

	while(1)
	{
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
	}

	return 0;
}

