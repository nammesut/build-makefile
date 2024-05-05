/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Apr 11, 2024
 *      Author: DELL
 */


#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Configuration Structure for GPIO
 * */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinType;									/* Select Pin mode In or Out */
	uint8_t GPIO_PinMode;									/* Select Pin mode In (Float, Analog, PuPd) or Out (PP, OD) */
	uint8_t GPIO_AltFunMode;
}GPIO_PinConfig_t;


/*
 * Handle Structure for GPIO
 * */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;



#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12 		12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15

#define GPIO_MODE_IN					0
#define GPIO_MODE_OUT_MEDIUM_SPEED		1
#define GPIO_MODE_OUT_LOW_SPEED			2
#define GPIO_MODE_OUT_HIGH_SPEED		3
#define GPIO_MODE_IT_RT					4
#define GPIO_MODE_IT_FT					5
#define GPIO_MODE_IT_RFT				6


#define GPIO_MODE_IN_ANALOG		0
#define GPIO_MODE_IN_FLOAT		1
#define GPIO_MODE_IN_PLPD		2

#define GPIO_MODE_OUT_PP			0
#define GPIO_MODE_OUT_OD			1
#define GPIO_MODE_OUT_ALTFUN_PP		2
#define GPIO_MODE_OUT_ALTFUN_OD		3



/**************************************************************************************
 * 									APIs GPIO
 **************************************************************************************/

/*
 * Peripheral Clock config
 * */

void GPIO_PeriCockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/*
 * GPIO Init - DeInit
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Read - write
 * */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ config and ISR handling
 * */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint32_t Priority, uint8_t PinNumber);
void GPIO_ISRHandling(uint8_t PinNumber);




#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
