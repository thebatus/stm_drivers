/*
 * stm32f072.h
 *
 *  Created on: May 19, 2023
 *      Author: Batuhan Ekmekcioglu
 */

#ifndef INC_STM32F072_H_
#define INC_STM32F072_H_
#define __vo volatile
#include <stdint.h>

/********************
 * processor spescific
 * ARM cortex M0
 */
//NVIC registers
#define NVIC_ISER  ((__vo uint32_t*)0xE000E100) //interrupt set
#define NVIC_ICER  ((__vo uint32_t*)0xE000E180) //interrupt clear
#define NVIC_ISPR  ((__vo uint32_t*)0xE000E200) //interrupt set pending
#define NVIC_ICPR  ((__vo uint32_t*)0xE000E280) //interrupt clear pending

//interrupt priority registers
#define NVIC_IPR_BADDR  ((__vo uint32_t*)0xE000E400)













//base addresses for flash and sram
#define FLASH_BADDR    0x08000000U  //flash memory
#define SRAM_BADDR     0x20000000U  //sram memory
#define ROM_BADDR	   0x1FFFC800U  //system memory

//base addresses for buses
#define APB_BADDR 	   0x40000000U   //apb base addr
#define AHB1_BADDR 	   0x40020000U   //ahb1 base addr
#define AHB2_BADDR 	   0x48000000U   //ahb2 base addr

//base addresses for peripherals
//AHB2
#define GPIOA_BADDR    (AHB2_BADDR+0x0000)
#define GPIOB_BADDR    (AHB2_BADDR+0x0400)
#define GPIOC_BADDR    (AHB2_BADDR+0x0800)
#define GPIOD_BADDR    (AHB2_BADDR+0x0C00)
#define GPIOE_BADDR    (AHB2_BADDR+0x1000)
#define GPIOF_BADDR    (AHB2_BADDR+0x1400)

//AHB1
#define DMA1_BADDR     (AHB1_BADDR+0x0000)
#define DMA2_BADDR     (AHB1_BADDR+0x0400)
#define RCC_BADDR      (AHB1_BADDR+0x1000)
#define FLASHINT_BADDR (AHB1_BADDR+0x2000)
#define CRC_BADDR      (AHB1_BADDR+0x3000)
#define TSC_BADDR      (AHB1_BADDR+0x4000)

//APB
#define TIM2_BADDR     (APB_BADDR+0x0000)
#define TIM3_BADDR     (APB_BADDR+0x0400)
#define TIM6_BADDR     (APB_BADDR+0x1000)
#define TIM7_BADDR     (APB_BADDR+0x1400)
#define TIM14_BADDR    (APB_BADDR+0x2000)
#define RTC_BADDR      (APB_BADDR+0x2800)
#define WWDG_BADDR     (APB_BADDR+0x2C00)
#define IWDG_BADDR     (APB_BADDR+0x2800)
#define SPI2_BADDR     (APB_BADDR+0x3800)
#define USART2_BADDR   (APB_BADDR+0x4400)
#define USART3_BADDR   (APB_BADDR+0x4800)
#define USART4_BADDR   (APB_BADDR+0x4C00)
#define USART5_BADDR   (APB_BADDR+0x5000)
#define I2C1_BADDR     (APB_BADDR+0x5400)
#define I2C2_BADDR     (APB_BADDR+0x5800)
#define USB_BADDR      (APB_BADDR+0x5C00)
#define UCSRAM_BADDR   (APB_BADDR+0x6000)
#define CAN_BADDR      (APB_BADDR+0x6400)
#define CRS_BADDR      (APB_BADDR+0x6C00)
#define PWR_BADDR      (APB_BADDR+0x7000)
#define DAC_BADDR      (APB_BADDR+0x7400)
#define CEC_BADDR      (APB_BADDR+0x7800)
#define SYSCFG_BADDR   0x40010000U
#define COMP_BADDR     (SYSCFG_BADDR+0x001C)
#define EXTI_BADDR     (SYSCFG_BADDR+0x0400)
#define USART6_BADDR   (SYSCFG_BADDR+0x1400)
#define USART7_BADDR   (SYSCFG_BADDR+0x1800)
#define USART8_BADDR   (SYSCFG_BADDR+0x1C00)
#define ADC_BADDR      (SYSCFG_BADDR+0x2400)
#define TIM1_BADDR     (SYSCFG_BADDR+0x2C00)
#define SPI1_BADDR     (SYSCFG_BADDR+0x3000
#define I2S_BADDR      (SPI1_BADDR+0x001C)
#define USART1_BADDR   (SYSCFG_BADDR+0x3800)
#define TIM15_BADDR    (SYSCFG_BADDR+0x4000)
#define TIM16_BADDR    (SYSCFG_BADDR+0x4400)
#define TIM17_BADDR    (SYSCFG_BADDR+0x4800)
#define DBGMCU_BADDR   (SYSCFG_BADDR+0x5800)


//gpio peripheral registers
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
	__vo uint32_t BRR;
}GPIO_reg;

//gpio
#define GPIOA  ((GPIO_reg*)GPIOA_BADDR)
#define GPIOB  ((GPIO_reg*)GPIOB_BADDR)
#define GPIOC  ((GPIO_reg*)GPIOC_BADDR)
#define GPIOD  ((GPIO_reg*)GPIOD_BADDR)
#define GPIOE  ((GPIO_reg*)GPIOE_BADDR)
#define GPIOF  ((GPIO_reg*)GPIOF_BADDR)

#define GPIO_BADDR_TO_CODE(x)  ((x==GPIOA)? 0:\
								(x==GPIOB)? 1:\
								(x==GPIOC)? 2:\
								(x==GPIOD)? 3:\
								(x==GPIOE)? 4:\
								(x==GPIOF)? 5:0)

//rcc peripheral registers
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
	__vo uint32_t CR2;

}RCC_reg;

#define RCC  ((RCC_reg*)RCC_BADDR)


//peripheral register definiton structure for EXTI
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_reg;

#define EXTI  ((EXTI_reg*)EXTI_BADDR)

//peripheral register definiton structure for SYSCFG
typedef struct{
	__vo uint32_t CFGR1;
	__vo uint32_t res;
	__vo uint32_t EXTICR1;
	__vo uint32_t EXTICR2;
	__vo uint32_t EXTICR3;
	__vo uint32_t EXTICR4;
	__vo uint32_t CFGR2;

}SYSCFG_reg;

#define SYSCFG ((SYSCFG_reg*) SYSCFG_BADDR)

//clock enable macros for gpio peripherals
#define GPIOA_CLK_EN()  (RCC->AHBENR |=(1<<17))
#define GPIOB_CLK_EN()  (RCC->AHBENR |=(1<<18))
#define GPIOC_CLK_EN()  (RCC->AHBENR |=(1<<19))
#define GPIOD_CLK_EN()  (RCC->AHBENR |=(1<<20))
#define GPIOE_CLK_EN()  (RCC->AHBENR |=(1<<21))
#define GPIOF_CLK_EN()  (RCC->AHBENR |=(1<<22))


//clock enable macros for i2c

//clock enable macros for spi

//clock enable macros for usart

//clock enable macros for syscfg
#define SYSCFG_CLK_EN() (RCC->APB2ENR |=(1<<0))


//clock disable macros for gpio peripherals
#define GPIOA_CLK_DIS()  (RCC->AHBENR &=~(1<<17))
#define GPIOB_CLK_DIS()  (RCC->AHBENR &=~(1<<18))
#define GPIOC_CLK_DIS()  (RCC->AHBENR &=~(1<<19))
#define GPIOD_CLK_DIS()  (RCC->AHBENR &=~(1<<20))
#define GPIOE_CLK_DIS()  (RCC->AHBENR &=~(1<<21))
#define GPIOF_CLK_DIS()  (RCC->AHBENR &=~(1<<22))


//clock disable macros for i2c

//clock disable macros for spi

//clock disable macros for usart

//clock disable macros for syscfg
#define SYSCFG_CLK_DIS() (RCC->APB2ENR &=~(1<<0))


//IRQ numbers for EXTI
#define IRQ_NO_EXTI0		5
#define IRQ_NO_EXTI1		5
#define IRQ_NO_EXTI2		6
#define IRQ_NO_EXTI3		6
#define IRQ_NO_EXTI4_15		7



//generic macros
#define ENABLE 		1
#define DISABLE 	0
#define SET 		ENABLE
#define RESET 		DISABLE













#endif /* INC_STM32F072_H_ */
