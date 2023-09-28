/*
 * stm32f411_gpio_driver.h
 *
 *  Created on: Sep 25, 2023
 *      Author: Quin
 */

#ifndef INC_STM32F411_GPIO_DRIVER_H_
#define INC_STM32F411_GPIO_DRIVER_H_

#include "stm32f411.h"
typedef struct
{
	//16 pin dùng 1byte đủ rồi
	uint8_t GPIO_PinNumber;       //possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;         //possible values from @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed;        //possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_Pin_PuPdControl; //possible values from @GPIO_PIN_IP_TYPE
	uint8_t GPIO_PinOPType;       //possible values from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunMode;   //possible values from @GPIO_PIN_AltFunmode
}GPIO_PinConfig_t;
typedef struct
{
//con trở chứa base address của gpio mà pin nó thuộc về ngoại vi đó
//ví dụ gọi pin 2 của gpioA thì phải trỏ tới gpioA
	GPIO_RegDef_t *pGPIOx;   //khai báo con trỏ kêu struct gpio_refdef_t, này là PORT
	GPIO_PinConfig_t GPIO_Pinconfig;
}GPIO_Handle_t; //ở trên là struct thanh ghi của gpio,còn ở dưới là cấu hính cho từng thanh ghi

//@GPIO_PIN_NUMBER
#define GPIO_PIN_NO_0         0
#define GPIO_PIN_NO_1         1
#define GPIO_PIN_NO_2         2
#define GPIO_PIN_NO_3         3
#define GPIO_PIN_NO_4         4
#define GPIO_PIN_NO_5         5
#define GPIO_PIN_NO_6         6
#define GPIO_PIN_NO_7         7
#define GPIO_PIN_NO_8         8
#define GPIO_PIN_NO_9         9
#define GPIO_PIN_NO_10        10
#define GPIO_PIN_NO_11        11
#define GPIO_PIN_NO_12        12
#define GPIO_PIN_NO_13        13
#define GPIO_PIN_NO_14        14
#define GPIO_PIN_NO_15        15

//@GPIO_PIN_MODE
#define GPIO_MODE_IN           0
#define GPIO_MODE_OUT          1
#define GPIO_MODE_ALTFN        2
#define GPIO_MODE_ANALOG       3
#define GPIO_MODE_IT_FT        4
#define GPIO_MODE_IT_RT        5
#define GPIO_MODE_IT_RFT       6
//@GPIO_PIN_SPEED
#define GPIO_SPEED_LOW         0
#define GPIO_SPEED_MEDIUM      1
#define GPIO_SPEED_FAST        2
#define GPIO_SPEED_HIGH        3
//@GPIO_PIN_IP_TYPE
#define GPIO_NO_PUPD           0
#define GPIO_PU                1
#define GPIO_PD                2
//@GPIO_PIN_OP_TYPE
#define GPIO_OP_TYPE_PP        0 //reset state
#define GPIO_OP_TYPE_OD        1



//peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init and De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);  //reset

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber); //đảo

//IRQ Configuration abd ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);


#endif /* INC_STM32F411_GPIO_DRIVER_H_ */
