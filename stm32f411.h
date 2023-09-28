/*
 * stm32f411.h
 *
 *  Created on: Sep 24, 2023
 *      Author: Quin
 */

#ifndef INC_STM32F411_H_
#define INC_STM32F411_H_

#include <stdint.h>
#define _vo volatile
/*
 *ARM Cortex Mx Processor NIVC ISERx register Address - Table 4-2 NVIC register summary
 */
#define NVIC_ISER0                                        ((_vo uint32_t*)0xE000E100)
#define NVIC_ISER1                                        ((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2                                        ((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3                                        ((_vo uint32_t*)0xE000E10C)
/*
 *ARM Cortex Mx Processor NIVC ICERx register Address - Table 4-2 NVIC register summary
 */
#define NVIC_ICER0                                        ((_vo uint32_t*)0xE000E180)
#define NVIC_ICER1                                        ((_vo uint32_t*)0xE000E184)
#define NVIC_ICER2                                        ((_vo uint32_t*)0xE000E188)
#define NVIC_ICER3                                        ((_vo uint32_t*)0xE000E18C)

/*
 *Arm Cortex Mx Processor Priority Register Address
 */
#define NVIC_PR_BASE_ADDR                                 ((_vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED                            4
//base address của flash và sram
#define FLASH_BASEADDR                                    0x08000000U    //main memory
#define SRAM1_BASEADDR                                    0x20000000U
#define ROM_BASEADDR                                      0x1FFF0000U    //system memory
#define SRAM_BASEADDR                                     SRAM1_BASEADDR

//bus peripheral base address
#define PERIPH_BASEADDR                                   0x40000000U
#define APB1PERIPH_BASEADDR                               PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR                               0x40010000U
#define AHB1PERIPH_BASEADDR                               0x40020000U
#define AHB2PERIPH_BASEADDR                               0x50000000U

//base address of peripheral hang on AHB1
#define GPIOA_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x1C00)

//base address off RCC
#define RCC_BASEADDR                                      (AHB1PERIPH_BASEADDR + 0x3800)
//base address of peripheral hang on APB1
#define I2C1_BASEADDR                                     (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                                     (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                                     (APB1PERIPH_BASEADDR + 0x5C00)
#define USART_BASEADDR                                    (APB1PERIPH_BASEADDR + 0x4400)
#define SPI2_BASEADDR                                     (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                                     (APB1PERIPH_BASEADDR + 0x3C00)

//base address of peripheral hang on APB2
#define USART1_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR                                     (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR                                     (APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR                                     (APB2PERIPH_BASEADDR + 0x5000)
#define SYSCFG_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR                                     (APB2PERIPH_BASEADDR + 0x3C00)

//define  struct of GPIO
typedef struct
{
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];
}GPIO_RegDef_t;

//define struct of RCC ->RCC register map
typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	uint32_t RESERVED1[2];
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	uint32_t RESERVED3[2];
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	uint32_t RESERVED5[2];
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	uint32_t RESERVED7[2];
	_vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
	uint32_t RESERVED8;
	_vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct {
	_vo uint32_t IMR; //interrupt mask register , thanh ghi cung cấp 1 bit cho mỗi line để cho phép che hay ko che các line tương ứng
	_vo uint32_t EMR; //tương tự cho khác sự kiện khác
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;//software interrupt event register - thanh ghi này dùng để kích hoạt software interrupt tương ứng,có thể sử dụng thay thế cho việc ngắt bằng hardware
	_vo uint32_t PR;  //pending regiter: mỗi khi có tín hiệu ngắt xảy ra)theo sường hoặc theo phần mềm) thì bit tương ứng trên thanh ghi này sẽ được set,khi ngắt đó được xử lí thì thôi
}EXTI_RegDef_t;

/*
 *định nghĩa thanh ghi cho SYSCFG-ROM
 */
typedef struct{
	_vo uint32_t MEMRMP;
	_vo uint32_t PMC;
	_vo uint32_t EXTICR[4];
	uint32_t     RESERVED1[2];
	_vo uint32_t CMPCR;
	uint32_t     RESERVED2[2];
	_vo uint32_t CEGR;
} SYSCFG_RegDef_t;



//peripheral definition
#define GPIOA      ((GPIO_RegDef_t*)GPIOA_BASEADDR)        //khai báo pointer kiểu struct GPIO_RegDef_t trở tới vùng địa chỉ của GPIOA
#define GPIOB      ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC      ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD      ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE      ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH      ((GPIO_RegDef_t*)GPIOH_BASEADDR)

//rcc definition
#define RCC        ((RCC_RegDef_t*)RCC_BASEADDR)         //khai báo pointer kiểu struct RCC_RefDef_t trỏ tới vùng địa chỉ của RCC_BASEADDR
#define EXTI       ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG     ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
//clock enable macro for GPIOx peripheral
//muon clock cho ngoại vi nao phai cấp clock cho bus mà ngoại vi đó treo lên
#define GPIOA_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 0))   //GPIO hang on AHB1, GPIOA la bit 0 của AHB1
#define GPIOB_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 7))

//clock enable macro for I2Cx peripheral
#define I2C1_PLCK_EN()                  (RCC->APB1ENR)|= (1 << 21))
#define I2C2_PLCK_EN()                  (RCC->APB1ENR)|= (1 << 22))
#define I2C3_PLCK_EN()                  (RCC->APB1ENR)|= (1 << 23))
//clock enable of SPI
#define SPI2_PLCK_EN()                  (RCC->APB1ENR |= (1 << 14))
#define SPI3_PLCK_EN()                  (RCC->APB1ENR |= (1 << 15))
#define SPI1_PLCK_EN()                  (RCC->APB2ENR |= (1 << 12))
#define SPI4_PLCK_EN()                  (RCC->APB2ENR |= (1 << 13))
#define SPI5_PLCK_EN()                  (RCC->APB2ENR |= (1 << 20))
//clock enable macro for USUART
#define USART2_PLCK_EN()                (RCC->APB1ENR |= (1 << 17))
#define USART1_PLCK_EN()                (RCC->APB2ENR |= (1 << 4))
#define USART6_PLCK_EN()                (RCC->APB2ENR |= (1 << 5))
//clock enable for SYSCFG
#define SYSCFG_PLCK_EN()                (RCC->APB2ENR |= (1 << 14))

//clock disable macro for gpio peripheral
#define GPIOA_PCLK_DI()                 (RCC->AHB1ENR &= ~(1 << 0))   //GPIO hang on AHB1, GPIOA la bit 0 của AHB1
#define GPIOB_PCLK_DI()                 (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()                 (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()                 (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()                 (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()                 (RCC->AHB1ENR &= ~(1 << 7))

//clock disable macro for I2Cx peripheral
#define I2C1_PLCK_DI()                  (RCC->APB1ENR)&= ~(1 << 21))
#define I2C2_PLCK_DI()                  (RCC->APB1ENR)&= ~(1 << 22))
#define I2C3_PLCK_DI()                  (RCC->APB1ENR)&= ~(1 << 23))
//clock disable of SPI
#define SPI2_PLCK_DI()                  (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PLCK_DI()                  (RCC->APB1ENR &= ~(1 << 15))
#define SPI1_PLCK_DI()                  (RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PLCK_DI()                  (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PLCK_DI()                  (RCC->APB2ENR &= ~(1 << 20))
//clock disable macro for USUART
#define USART2_PLCK_DI()                (RCC->APB1ENR &= ~(1 << 17))
#define USART1_PLCK_DI()                (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PLCK_DI()                (RCC->APB2ENR &= ~(1 << 5))
//clock disable for SYSCFG
#define SYSCFG_PLCK_DI()                (RCC->APB2ENR &= ~(1 << 14))

//macro reset peripheral GPIOx
#define GPIOA_REG_RESET()               do{RCC->AHB1RSTR |= (1 << 0);RCC->AHB1RSTR &= ~(1 << 0);}while(0)  //while(0) thực hiện 1 lần
#define GPIOB_REG_RESET()               do{RCC->AHB1RSTR |= (1 << 1);RCC->AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_REG_RESET()               do{RCC->AHB1RSTR |= (1 << 2);RCC->AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_REG_RESET()               do{RCC->AHB1RSTR |= (1 << 3);RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_REG_RESET()               do{RCC->AHB1RSTR |= (1 << 4);RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOH_REG_RESET()               do{RCC->AHB1RSTR |= (1 << 7);RCC->AHB1RSTR &= ~(1 << 7);}while(0)

/*
 *macro trả về code (từ 0-7) for base address(x) của ngoại vi gpio)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ((x == GPIOA)?0:\
		                               (x == GPIOB)?1:\
		                               (x == GPIOC)?2:\
		                               (x == GPIOB)?3:\
		                               (x == GPIOA)?4:\
		                               (x == GPIOB)?5:\
		                               (x == GPIOA)?6:\
		                               (x == GPIOB?7:0))
//interrupt request number
#define IRQ_NO_EXTI0              6
#define IRQ_NO_EXTI1              7
#define IRQ_NO_EXTI2              8
#define IRQ_NO_EXTI3              9
#define IRQ_NO_EXTI4              10
#define IRQ_NO_EXTI9_5            23
#define IRQ_NO_EXTI15_10          40

//một vài macro chung
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#endif /* INC_STM32F411_H_ */
