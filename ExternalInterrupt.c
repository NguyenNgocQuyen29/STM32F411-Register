/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************

	1. Enable the SYSCFG(F4)/AFIO(F1) bit in RCC register
	2. Configure the EXTI configuration Register in the SYSCFG/AFIO
	3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt

********************************************************/
//co the tao struct cũng được nhưng làm cái này thử thôi :))))
//địa chỉ clock GPIOA,EXTI
uint32_t volatile *pClkAhb1 = (uint32_t*)(0x40020000 + 0x30);
uint32_t volatile *pClkApb2 = (uint32_t*)(0x40010000 + 0x44);
//địa chỉ config GPIO với EXTI(IMR-mặt nạ,FTSR-xườn lên,PR-pending)
uint32_t volatile *pGPIOAMode = (uint32_t*)(0x40020000 + 0x00);//moder
uint32_t volatile *pGPIOAPullUp = (uint32_t*)(0x40020000 + 0x0C);//PUPDR

uint32_t volatile *pEXTIMask = (uint32_t*)(0x40013C00 + 0x00);
uint32_t volatile *pEXTIEdgeFall = (uint32_t*)(0x40013C00 + 0x0C);
uint32_t volatile *pEXTIPend = (uint32_t*)(0x40013C00 + 0x14);
//địa chỉ congif NVIC
uint32_t volatile *pNVICIRQEn = (uint32_t*)(0xE000E100);//RM Cortex Mx Processor NIVC ISERx register Address - Table 4-2 NVIC register summary
void Config(){
	//cấu hình cấp clock cho GPIOA và EXTI
	*pClkAhb1 |= (1 << 0);
	*pClkApb2 |= (1 << 14);
	//cấu hình cho mode A0(input,pull up)
	*pGPIOAMode &= ~(1 << 0);
	*pGPIOAPullUp |= (1 << 0)
	//cấu hình EXTI cho phép line tương ứng - sườn lên
	*pEXTIMask |= (1 << 0);	//tắt mặt nạ
	*pEXTIEdgeFall |= (1 << 0);
	//cấu hình enable NVIC
	*pNVICIRQEn |= (1 << 6);//IRQ_NO_EXTI0              6
}
/*************>>>>>>> STEPS FOLLOWED - IRQ HANDLER <<<<<<<<************

1. Check the Pin, which trgerred the Interrupt
2. Clear the Interrupt Pending Bit

********************************************************/



int main(void)
{

}
void EXTI0_IRQHandler(void){
	/*Do something you line
	 * Example: Set a flag to use in main*/
	flag_button = 1;
	//clear Pending by setting Pending bit
	*pEXTIPend |= (1 << 0);
}