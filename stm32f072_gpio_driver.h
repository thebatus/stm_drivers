/*
 * stm32f072_gpio_driver.h
 *
 *  Created on: 20 May 2023
 *      Author: Batuhan Ekmekcioglu
 */

#ifndef INC_STM32F072_GPIO_DRIVER_H_
#define INC_STM32F072_GPIO_DRIVER_H_

#include "stm32f072.h"
#include <stdint.h>

typedef struct{

	uint8_t GPIO_pinNumber;
	uint8_t GPIO_pinMode;
	uint8_t GPIO_pinSpeed;
	uint8_t GPIO_pinPuPd;
	uint8_t GPIO_pinOutType;
	uint8_t GPIO_pinAFMode;

}GPIO_pinConfig;




//handle structure for gpio pin
typedef struct{

	GPIO_reg *pGPIOx;  //holds the base address of the gpio port which the pin belongs. başındaki küçük p pointer olduğunu belirtmek için
	GPIO_pinConfig GPIO_pinConfig; //hold gpio pin config settings

}GPIO_handle;

/*
 * gpio pin possible modes
 */
#define GPIO_MODE_INPUT  	0
#define GPIO_MODE_OUTPUT  	1
#define GPIO_MODE_AF  		2
#define GPIO_MODE_ANALOG  	3
#define GPIO_MODE_IT_FT 	4  //input falling edge  interrupt modes
#define GPIO_MODE_IT_RT  	5  //input rising edge
#define GPIO_MODE_IT_RFT	6  //both rising and falling edge


/*
 * gpio output types
 */
#define GPIO_OTYPE_PP 	0
#define GPIO_OTYPE_OD 	1


/*
 * gpio speed
 */
#define GPIO_SPEED_LOW 	0
#define GPIO_SPEED_MED 	1
#define GPIO_SPEED_HIGH	3

/*
 * gpio pull up/pull down
 */
#define GPIO_NOPUPD 	0
#define GPIO_PU 		1
#define GPIO_PD		 	2

/*
 * gpio pin numbers
 */
#define GPIO_PIN0	0
#define GPIO_PIN1	1
#define GPIO_PIN2	2
#define GPIO_PIN3	3
#define GPIO_PIN4	4
#define GPIO_PIN5	5
#define GPIO_PIN6	6
#define GPIO_PIN7	7
#define GPIO_PIN8	8
#define GPIO_PIN9	9
#define GPIO_PIN10	10
#define GPIO_PIN11	11
#define GPIO_PIN12	12
#define GPIO_PIN13	13
#define GPIO_PIN14	14
#define GPIO_PIN15	15

/*
 * gpio alternate functions
 */
#define GPIO_AF0 	0
#define GPIO_AF1 	1
#define GPIO_AF2 	2
#define GPIO_AF3 	3
#define GPIO_AF4 	4
#define GPIO_AF5 	5
#define GPIO_AF6 	6
#define GPIO_AF7 	7








/* API's supported by this driver */

/**********************************************************************
 * @fn				-GPIO_PeriClkControl
 *
 * @brief			-enables or disables the peripheral clock for the given gpio port
 *
 * @param[in]		-base address of gpio peripheral
 * @param[in]		-enable or disable
 *
 * @return			-none
 *
 */
void GPIO_PeriClkControl(GPIO_reg *pGPIOx, uint8_t en);



/**********************************************************************
 * @fn				-GPIO_Init
 *
 * @brief			-initializes the gpio
 *
 * @param[in]		-gpio handle structure which involves configuring the pin for given gpio port
 *
 * @return			-none
 *
 */
void GPIO_Init(GPIO_handle *pGPIOHandle);




/**********************************************************************
 * @fn				-GPIO_DeInit
 *
 * @brief			-deinitializes the gpio
 *
 * @param[in]		-pointer for base address of the gpio port
 *
 * @return			-none
 *
 * @note			-only needs the base address of the gpio port because it will deinitialize by using the peripheral reset registers in rcc
 *
 */
void GPIO_DeInit(GPIO_reg *pGPIOx);


/**********************************************************************
 * @fn				-GPIO_ReadPin
 *
 * @brief			-reads the value from the given pin for the given gpio port
 *
 * @param[in]		-base address of gpio peripheral
 * @param[in]		-pin number
 *
 * @return			-read value
 *
 */

uint8_t GPIO_ReadPin(GPIO_reg *pGPIOx, uint8_t pinNum);
uint16_t GPIO_ReadPort(GPIO_reg *pGPIOx);
void GPIO_WritePin(GPIO_reg *pGPIOx, uint8_t pinNum, uint8_t value);
void GPIO_WritePort(GPIO_reg *pGPIOx, uint16_t value);void GPIO_TogglePin(GPIO_reg *pGPIOx, uint8_t pinNum);

void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t en);
void GPIO_IRQHandle(uint8_t pinNum);
void GPIO_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNum);














#endif /* INC_STM32F072_GPIO_DRIVER_H_ */
