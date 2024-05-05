/*
 * stm32f103xx.h
 *
 *  Created on: Apr 10, 2024
 *      Author: DELL
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_


#include <stdint.h>
#include "stm32f103xx_gpio_driver.h"


#define __vo volatile


#define FLASH_BASEADDR					0x08000000U
#define SRAM_BASEADDR					0x20000000U			/* 96 Kbytes of static SRAM */
#define ROM_BASEADDR					0x1FFFB000U




#define PERIPH_BASEADDR						0x40000000U
#define AHBPERIPH_BASEADDR					0x40018000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U



#define GPIOA_BASEADDR						(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR						(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR						(APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR						(APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x0400)
#define AFIO_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)

#define RCC_BASEADDR						0x40021000U

#define NVIC_ISER0							((__vo uint32_t*)	0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)	0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)	0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)	0xE000E10C)

#define NVIC_ICER0							((__vo uint32_t*)	0XE000E180)
#define NVIC_ICER1							((__vo uint32_t*)	0XE000E184)
#define NVIC_ICER2							((__vo uint32_t*)	0XE000E188)
#define NVIC_ICER3							((__vo uint32_t*)	0XE000E18C)

#define NVIC_IPR							((__vo uint32_t*)	0xE000E400)

#define NO_PR_BITS 			4



/* Structure RCC */
typedef struct {
	__vo uint32_t	CR;					/* Clock control register 							Addr offset: 0x00 */
	__vo uint32_t	CFGR;				/* Clock configuration register 					Addr offset: 0x04 */
	__vo uint32_t	CIR;				/* Clock interrupt register 						Addr offset: 0x08 */
	__vo uint32_t	APB2RSTR;			/* APB2 peripheral reset register 					Addr offset: 0x0C */
	__vo uint32_t	APB1RSTR;			/* APB1 peripheral reset register 					Addr offset: 0x10 */
	__vo uint32_t	AHBENR;				/* AHB peripheral clock enable register		 		Addr offset: 0x14 */
	__vo uint32_t	APB2ENR;			/* APB2 peripheral clock enable register		 	Addr offset: 0x18 */
	__vo uint32_t	APB1ENR;			/* APB1 peripheral clock enable register		 	Addr offset: 0x1C */
	__vo uint32_t	BDCR;				/* Backup domain control register		 			Addr offset: 0x20 */
	__vo uint32_t	CSR;				/* Control/status register		 					Addr offset: 0x24 */
}RCC_RegDef_t;

/* Structure GPIO */
typedef struct {
	__vo uint32_t	CR[2];				/* Port configuration register [low, high] 			Addr offset: 0x00-0x04 */
	__vo uint32_t	IDR;				/* Port input data register 						Addr offset: 0x08 */
	__vo uint32_t	ODR;				/* Port output data register 						Addr offset: 0x0C */
	__vo uint32_t	BSRR;				/* Port bit set/reset register 						Addr offset: 0x10 */
	__vo uint32_t	BRR;				/* Port bit reset register 							Addr offset: 0x14 */
	__vo uint32_t	LCKR;				/* Port configuration lock register		 			Addr offset: 0x18 */
}GPIO_RegDef_t;

/* Structure SPI */
typedef struct {
	__vo uint32_t	CR1;				/* SPI control register 1				 			Addr offset: 0x00 */
	__vo uint32_t	CR2;				/* SPI control register 2	 						Addr offset: 0x04 */
	__vo uint32_t	SR;					/* SPI status register		 						Addr offset: 0x08 */
	__vo uint32_t	DR;					/* SPI data register 								Addr offset: 0x0C */
	__vo uint32_t	CRCPR;				/* SPI CRC polynomial register						Addr offset: 0x10 */
	__vo uint32_t	RXCRCR;				/* SPI RX CRC register					 			Addr offset: 0x14 */
	__vo uint32_t	TXCRCR;				/* SPI TX CRC register					 			Addr offset: 0x18 */
	__vo uint32_t	I2SCFGR;			/* SPI_I2S configuration register		 			Addr offset: 0x1C */
	__vo uint32_t	I2SPR;				/* SPI_I2S prescaler register			 			Addr offset: 0x20 */
}SPI_RegDef_t;

/* Structure I2C */
typedef struct {
	__vo uint32_t	CR1;				/* I2C control register 1				 			Addr offset: 0x00 */
	__vo uint32_t	CR2;				/* I2C control register 2	 						Addr offset: 0x04 */
	__vo uint32_t	OAR1;				/* I2C Own address register 1		 				Addr offset: 0x08 */
	__vo uint32_t	OAR2;				/* I2C Own address register 2						Addr offset: 0x0C */
	__vo uint32_t	DR;					/* I2C Data register								Addr offset: 0x10 */
	__vo uint32_t	SR1;				/* I2C Status register 1				 			Addr offset: 0x14 */
	__vo uint32_t	SR2;				/* I2C Status register 2				 			Addr offset: 0x18 */
	__vo uint32_t	CCR;				/* I2C Clock control register			 			Addr offset: 0x1C */
	__vo uint32_t	TRISE;				/* I2C TRISE register					 			Addr offset: 0x20 */
}I2C_RegDef_t;

/* Struct EXTI */
typedef struct {
	__vo uint32_t	IMR;				/* Interrupt mask register				 			Addr offset: 0x00 */
	__vo uint32_t	EMR;				/* Event mask register		 						Addr offset: 0x04 */
	__vo uint32_t	RTSR;				/* Rising trigger selection register 				Addr offset: 0x08 */
	__vo uint32_t	FTSR;				/* Falling trigger selection register				Addr offset: 0x0C */
	__vo uint32_t	SWIER;				/* Software interrupt event register				Addr offset: 0x10 */
	__vo uint32_t	PR;					/* Pending register						 			Addr offset: 0x14 */
}EXTI_RegDef_t;

/* Struct AFIO */
typedef struct {
	__vo uint32_t	EVCR;				/* Event control register				 							Addr offset: 0x00 */
	__vo uint32_t	MAPR;				/* AF remap and debug I/O configuration register		 			Addr offset: 0x04 */
	__vo uint32_t	EXTICR[4];			/* External interrupt configuration register[0-3]				 	Addr offset: 0x08-0x14 */
	__vo uint32_t	MAPR2;				/* AF remap and debug I/O configuration register2					Addr offset: 0x1C */
}AFIO_RegDef_t;


/* Structure base address peripheral */
#define GPIOA					((GPIO_RegDef_t*)	GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)	GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)	GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)	GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)	GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)	GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)	GPIOG_BASEADDR)

#define RCC						((RCC_RegDef_t*)	RCC_BASEADDR)

#define SPI1					((SPI_RegDef_t*)	SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)	SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)	SPI3_BASEADDR)

#define I2C1					((I2C_RegDef_t*)	I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)	I2C2_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)	EXTI_BASEADDR)
#define AFIO					((AFIO_RegDef_t*)	AFIO_BASEADDR)


/* Clock Enable for GPIOx Peripheral */
#define GPIOA_PCLK_EN()				(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()				(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()				(RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()				(RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()				(RCC->APB2ENR |= (1 << 8))

/* Clock Disable for GPIOx Peripheral */
#define GPIOA_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 8))

/* Reset register for GPIOx Peripheral */
#define GPIOA_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8)); }while(0)

/* Clock Enable for SPIx Peripheral */
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))

/* Clock Disable for SPIx Peripheral */
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))

/* Clock Enable for I2Cx Peripheral */
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))

/* Clock Disable for I2Cx Peripheral */
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))

/* Clock Enable for UARTx Peripheral */
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))

/* Clock Disable for UARTx Peripheral */
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))


#define AFIO_PCLK_EN()				(RCC->APB2ENR |= (1 << 0))


#define GPIO_BASEADDR_TO_CODE(x)			( 	(x == GPIOA) ? 0 : \
												(x == GPIOB) ? 1 : \
												(x == GPIOC) ? 2 : \
												(x == GPIOD) ? 3 : \
												(x == GPIOE) ? 4 : \
												(x == GPIOF) ? 5 : \
												(x == GPIOG) ? 6 : 0 )


#define EXTI_IRQ_NO_0 			6
#define EXTI_IRQ_NO_1 			7
#define EXTI_IRQ_NO_2 			8
#define EXTI_IRQ_NO_3 			9
#define EXTI_IRQ_NO_4 			10
#define EXTI_IRQ_NO_5_9 		23
#define EXTI_IRQ_NO_10_15 		40


/* Clock enable for EXTI Peripheral */


#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET 				SET
#define GPIO_PIN_RESET				RESET


#endif /* INC_STM32F103XX_H_ */
