/*
 * stm32f411disc_gpio_driver.c
 *
 *  Created on: Sep 25, 2023
 *      Author: Quin
 */

#include "stm32f411_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
	   }
       }
}
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	//1.Config the mode of gpio pin
	uint32_t temp = 0;
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));//trạng thái thanh mode,cần ít nhất 2 bit để config cho nó,nên mỗi lần config phải dịch trái 2 bit
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);            //xóa xong mới ghi giá trị vào thanh ghi moder, &= ~(0x03)tức là 0011 vì có 2 bit nên phải clear 2 bit
		pGPIOHandle->pGPIOx->MODER |= temp;  //ghi trạng thái vào moder
	}
	else{
		//interrupt
		if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR-Falling Trigger selection register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit - xóa bit RTSR tương ừng
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1. configure the RTSR-Rising trigger selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
						//clear the corresponding FTSR bit - xóa bit FTSR tương ừng
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. congifure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit - xóa bit FTSR tương ừng
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		}
		//2. configure the gpio port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PLCK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		//3. Enable the ext interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber;
	}
	temp = 0;

	//2.Config speed
	temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));//trạng thái thanh mode,cần ít nhất 2 bit để config cho nó,nên mỗi lần config phải dịch trái 2 bit
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));            //xóa xong mới ghi giá trị vào thanh ghi speed
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;  //ghi trạng thái vào moder

	temp = 0;
	//3. Config pu-pd settings
	temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_Pin_PuPdControl << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));//trạng thái thanh mode,cần ít nhất 2 bit để config cho nó,nên mỗi lần config phải dịch trái 2 bit
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));            //xóa xong mới ghi giá trị vào thanh ghi input pull up/pull down
	pGPIOHandle->pGPIOx->PUPDR |= temp;  //ghi trạng thái vào pu-pd

	temp = 0;

	//4.Config optype
	temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPType << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));//trạng thái thanh mode,cần ít nhất 2 bit để config cho nó,nên mỗi lần config phải dịch trái 2 bit
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x03 << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));            //xóa xong mới ghi giá trị vào thanh ghi input pull up/pull down
	pGPIOHandle->pGPIOx->OTYPER |= temp;  //ghi trạng thái vào optype

	temp = 0;

	//5.Config the alt functionality
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber /8; //high~AFR[1] , low~AFR[0} , nếu mà chia cho 8 bằng 0 là thanh ghi AFRL ,bằng 1 là AFRH
		temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber % 8; //xác định pin thứ mấy của thanh ghi
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));  //thanh ghi này 1 pin có 4 bit nên nhân 4
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber << (4*temp2)); //ghi giá trị vào thanh ghi
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
		}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
		}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
		}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
		}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
		}
}
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber){ //IDR: input data register
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> GPIO_PinNumber)& 0x00000001); //đọc bit nào thì dịch về bit 0 rồi &1 để tất cả về 0 chỉ còn bit 0 là 1 để đọc giá trị trả về
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = pGPIOx->IDR; //trỏ nguyên port
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << GPIO_PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << GPIO_PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value){
	pGPIOx->ODR=value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber){
	pGPIOx->ODR ^= (1 << GPIO_PinNumber);    //^: xor giống nhau về 0,khác về 1
}


//IRQ Configuration abd ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//documents: Cortex-M4 Devices -> Nested Vectored Interrupt Controller
		if(IRQNumber <= 31){
			//chương trình cho ISER0 - Interrupt Set-Enable register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber%32));  //chia lấy phần dư ,dư bao nhiêu thì dịch bấy nhiêu bit trên thanh ghi này
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	}else
	{
		if(IRQNumber <= 31){
			//chương trình cho ICER0 - Interrupt Clear-Enable register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber%32));  //chia lấy phần dư ,dư bao nhiêu thì dịch bấy nhiêu bit trên thanh ghi này
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	//1. first lét find outIRQNumber the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section =  IRQNumber% 4;
	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr regisster corresponfing to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
	  //clear
		EXTI->PR |= (1 << PinNumber);
	}
}
