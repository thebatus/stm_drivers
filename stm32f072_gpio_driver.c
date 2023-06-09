/*
 * stm32f072_gpio_driver.c
 *
 *  Created on: 20 May 2023
 *      Author: Batuhan Ekmekcioglu
 */

#include "stm32f072_gpio_driver.h"
#include "stm32f072.h"



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
void GPIO_PeriClkControl(GPIO_reg *pGPIOx, uint8_t en){

	if(en==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_CLK_EN();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_CLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_CLK_EN();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_CLK_EN();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_CLK_EN();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_CLK_EN();
		}
	}
	else{
		if(pGPIOx==GPIOA){
			GPIOA_CLK_DIS();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_CLK_DIS();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_CLK_DIS();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_CLK_DIS();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_CLK_DIS();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_CLK_DIS();
		}
	}
}




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
void GPIO_Init(GPIO_handle *pGPIOHandle){

	//mode
	uint32_t temp=0;

	if(pGPIOHandle->GPIO_pinConfig.GPIO_pinMode <= GPIO_MODE_ANALOG){ //interrupt modes start after analog mode.
		temp=(pGPIOHandle->GPIO_pinConfig.GPIO_pinMode << (2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber));
		//pinconfig pointer değil o yüzden . ile ulaştık
		//her pin 2 bitlik yer kapladığı için ikiyle çarparak ne kadar shift edilceği, shift edilen değer de yazılması istenen mode değeri
		pGPIOHandle->pGPIOx->MODER&=~(0x3<<2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->MODER|=temp;
		temp=0;
	}
	else{
		//interrupt mode
		/*
		 * bu kısım peripheral ile ilgili olan interrupt kısmıydı
		 * processor ile ilgili olan  kısım IRQconfig fonksiyonunda ele alınacak
		 */
		if(pGPIOHandle->GPIO_pinConfig.GPIO_pinMode == GPIO_MODE_IT_FT){

			//1.configure the FTSR(falling trigger selection register)
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
			//clear the rtsr bit to avoid errors
			EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);


		}
		else if(pGPIOHandle->GPIO_pinConfig.GPIO_pinMode == GPIO_MODE_IT_RT){
			//1.configure the RTSR
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
			//clear the ftsr bit to avoid errors
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);

		}
		else if(pGPIOHandle->GPIO_pinConfig.GPIO_pinMode == GPIO_MODE_IT_RFT){
			//1.configure both FTSR and RTSR
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);

			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);

		}

		//2.configure the gpio port selection on SYSCFG_EXTICR

		uint8_t temp1=pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber/4; //hangi exticr registerında olcağı
		uint8_t temp2=pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber%4; //hangi pin olcağı. her pin 4 bit kaplıyor
		uint8_t port = GPIO_BADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_CLK_EN();
		if(temp1==0){
			SYSCFG->EXTICR1|=port<<(temp2*4);
		}
		else if(temp1==1){
			SYSCFG->EXTICR2|=port<<(temp2*4);
		}
		else if(temp1==2){
			SYSCFG->EXTICR3|=port<<(temp2*4);
		}
		else if(temp1==3){
			SYSCFG->EXTICR4|=port<<(temp2*4);
		}
		//3.enable exti interrupt delivery using IMR (interrupt mask register)

		EXTI->IMR|=(1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	}

	//speed
	temp=(pGPIOHandle->GPIO_pinConfig.GPIO_pinSpeed)<<(2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR&=~(0x3<<2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	//pupd
	temp=(pGPIOHandle->GPIO_pinConfig.GPIO_pinPuPd)<<(2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->PUPDR&=~(0x3<<2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->PUPDR|=temp;
	temp=0;

	//output type
	temp=(pGPIOHandle->GPIO_pinConfig.GPIO_pinOutType)<<(pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OTYPER&=~(0x3<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OTYPER|=temp;
	temp=0;

	//alternate function
	if(pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber<=7){
		temp=(pGPIOHandle->GPIO_pinConfig.GPIO_pinAFMode)<<(4*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->AFRL&=~(0xF<<4*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->AFRL|=temp;
		temp=0;
	}
	else{
		temp=(pGPIOHandle->GPIO_pinConfig.GPIO_pinAFMode)<<(4*((pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber)-8));
		pGPIOHandle->pGPIOx->AFRH&=~(0xF<<4*((pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber)-8));
		pGPIOHandle->pGPIOx->AFRH|=temp;
		temp=0;
	}

}



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
void GPIO_DeInit(GPIO_reg *pGPIOx){
	if(pGPIOx==GPIOA){
		RCC->AHBRSTR|=(1<<17);
		RCC->AHBRSTR&=~(1<<17);
	}
	else if(pGPIOx==GPIOB){
		RCC->AHBRSTR|=(1<<18);
		RCC->AHBRSTR&=~(1<<18);
	}
	else if(pGPIOx==GPIOC){
		RCC->AHBRSTR|=(1<<19);
		RCC->AHBRSTR&=~(1<<19);
	}
	else if(pGPIOx==GPIOD){
		RCC->AHBRSTR|=(1<<20);
		RCC->AHBRSTR&=~(1<<20);
	}
	else if(pGPIOx==GPIOE){
		RCC->AHBRSTR|=(1<<21);
		RCC->AHBRSTR&=~(1<<21);
	}
	else if(pGPIOx==GPIOF){
		RCC->AHBRSTR|=(1<<22);
		RCC->AHBRSTR&=~(1<<22);
	}
}




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
uint8_t GPIO_ReadPin(GPIO_reg *pGPIOx, uint8_t pinNum){

	uint8_t value;
	value=(uint8_t)(pGPIOx->IDR>>pinNum) & 0x00000001;
	return value;
}


uint16_t GPIO_ReadPort(GPIO_reg *pGPIOx){

	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}




/**********************************************************************
 * @fn				-GPIO_WritePin
 *
 * @brief			-writes a value to the given pin for the given gpio port
 *
 * @param[in]		-base address of gpio peripheral
 * @param[in]		-pin number
 * @param[in]		-value to write
 *
 * @return			-none
 *
 */
void GPIO_WritePin(GPIO_reg *pGPIOx, uint8_t pinNum, uint8_t value){

	pGPIOx->ODR&=~(1<<pinNum);
	pGPIOx->ODR|=(value<<pinNum);
}



void GPIO_WritePort(GPIO_reg *pGPIOx, uint16_t value){

	pGPIOx->ODR=value;
}



void GPIO_TogglePin(GPIO_reg *pGPIOx, uint8_t pinNum){

	pGPIOx->ODR^=(1<<pinNum); //toggle için xor
}


void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t en){	//processor specific

	if(en==ENABLE){

		*NVIC_ISER|=(1<<IRQNum);
	}
	else{
		*NVIC_ICER|=(1<<IRQNum);
	}

}


void GPIO_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNum){

	uint8_t iprx=IRQNum/4; //her peri 8 bitlik 4 kısma bölündüğü için hangi peride olduğunu bulmak için /4
	uint8_t ipr_section=IRQNum%4;
	*(NVIC_IPR_BADDR+iprx)|=(IRQPriority<<(8*ipr_section+4));
	/*+iprx diyerek ilgili kısmın adresine gittik
	 * uint32_t yi 1 artırdığımız zaman adres 4 artar
	 *8 ile çarptık çünkü 8 bitlik 4 kısımdan oluşuyor her register
	 *8 bitin 0-3 arası tanımlanmamış,4-8 arası priority belirlemek için.
	 *bu yüzden 4 ile topladık
	 */
}


void GPIO_IRQHandle(uint8_t pinNum){

	//1. clear the exti pr for corresponding pin number

	if(EXTI->PR & (1<<pinNum)){
		EXTI->PR |= (1<<pinNum); //1 yazınca clearlanır bu registerın özelliği
	}
}








