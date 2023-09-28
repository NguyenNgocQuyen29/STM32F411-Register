/*
 * 001_Toggle_Led.c
 *
 *  Created on: Sep 26, 2023
 *      Author: Quin
 */

#include "stm32f411_gpio_driver.h"
#include "stm32f411.h"

#define HIGH            1
#define BT_PRESSED      HIGH
void delay(void){
	for(int i=0;i<50000; i++);
}
int main(void){
	GPIO_Handle_t GpioLed, GpioButton;
	//this is gpio config
	GpioLed.pGPIOx = GPIOD; //pGPIOx chọn port ,còn kia cấu hình thanh ghi
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_Pin_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE); //truyền vào 2 thống sô đó là baseaddress của ngoại vi và enable
	GPIO_Init(&GpioLed); //truyền vào địa chỉ

	//this is button config , button của board này là chân PA0
	GpioButton.pGPIOx = GPIOA; //pGPIOx chọn port ,còn kia cấu hình thanh ghi
	GpioButton.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioButton.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; INPUT thì không cần sử dụng
	GpioButton.GPIO_Pinconfig.GPIO_Pin_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE); //truyền vào 2 thống sô đó là baseaddress của ngoại vi và enable
	GPIO_Init(&GpioButton); //truyền vào địa chỉ

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BT_PRESSED){   //bình thường là low,khi nhấn nút là high
			delay(); //để delay ở trên để chống dội phím
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
	return 0;
}
