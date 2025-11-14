/**
  ******************************************************************************
  * @file    hardware_stm_gpio.c 
  * @author  mortamar@andrew.cmu.edu
  * @version 1.0
  * @date    Septembr-2021
  * @brief   Controls STM32F446ze GPIO
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "stm32f4xx_rcc_mort.h"
#include <cstdint>


//led 1 is connected to PB0. 
// GPIO B addresses: 0x4002 0400 - 0x4002 07FF
// GPIO C addresses: 0x4002 0800 - 0x4002 0BFF


/* MACRO definitions----------------------------------------------------------*/
//Port B addresses:
#define PORTB_BASE_ADDRESS ((uint32_t)0x40020400)        //The first address in memory corresponding to Port B (this is in the user manual!)
// I gave you the first one, now you fill in the rest, check in the user manual what is the offset from the base address for each register!
#define PORTB_MODER_REGISTER (PORTB_BASE_ADDRESS + 0x00) //replace the question mark with the correct offset!
#define PORTB_OTYPER_REGISTER (PORTB_BASE_ADDRESS + 0x04)
#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)
#define PORTB_PUPDR_REGISTER (PORTB_BASE_ADDRESS + 0x0C)
#define PORTB_IDR_REGISTER (PORTB_BASE_ADDRESS + 0x10)
#define PORTB_ODR_REGISTER (PORTB_BASE_ADDRESS + 0x14)
//#define PORTB_BSRRL_REGISTER (PORTB_BASE_ADDRESS + 0x18)
#define PORTB_BSRR_REGISTER (PORTB_BASE_ADDRESS + 0x18)
//#define PORTB_BSRRH_REGISTER (PORTB_BASE_ADDRESS + 0x1A)
//#define PORTB_LCKR_REGISTER (PORTB_BASE_ADDRESS + 0x1C)
#define PORTB_AFR1_REGISTER (PORTB_BASE_ADDRESS + 0x20)
#define PORTB_AFR2_REGISTER (PORTB_BASE_ADDRESS + 0x24)
//#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)

//Port C addresses:
#define PORTC_BASE_ADDRESS      ((uint32_t)0x40020800)
#define PORTC_MODER_REGISTER    (PORTC_BASE_ADDRESS + 0x00)
#define PORTC_OTYPER_REGISTER   (PORTC_BASE_ADDRESS + 0x04)
#define PORTC_OSPEEDR_REGISTER  (PORTC_BASE_ADDRESS + 0x08)
#define PORTC_PUPDR_REGISTER    (PORTC_BASE_ADDRESS + 0x0C)
#define PORTC_IDR_REGISTER      (PORTC_BASE_ADDRESS + 0x10)
#define PORTC_ODR_REGISTER      (PORTC_BASE_ADDRESS + 0x14)
#define PORTC_BSRR_REGISTER     (PORTC_BASE_ADDRESS + 0x18)
#define PORTC_AFRL_REGISTER     (PORTC_BASE_ADDRESS + 0x20)
#define PORTC_AFRH_REGISTER     (PORTC_BASE_ADDRESS + 0x24)

//Some Masks
#define _2BIT_FIELD(pin)        (0x3u << ((pin) * 2))
#define _1BIT_FIELD(pin)        (1u    << (pin))

//so we can write a specific 2-bit value (00..11) at a pin’s field:
#define _2BIT_VAL(pin, v) ((uint32_t)((v) & 0x3u) << ((pin) * 2))
//flags MODER Register:

//flags OTYPER Register:

//flags OSPEEDR Register:

//flags PUPDR Register:

//input data register:

//flags AFR1 Register:

//flags ODR Register:


/* function definitions----------------------------------------------------------*/

void initGpioC6AsInput( void )
{
    uint32_t  * reg_pointer; 
    /* GPIOC Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* GPIOC Pin 6 as input: MODER[13:12] = 00 */
    reg_pointer = (uint32_t*)PORTC_MODER_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_2BIT_FIELD(6);
    
    /*PUSH-PULL Pin*/ /*OTYPER[6] = 0 */
    reg_pointer = (uint32_t*)PORTC_OTYPER_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_1BIT_FIELD(6);

    /*GPIOC pin 6 high speed OSPEEDR[13:12] = 10*/
    reg_pointer = (uint32_t*)PORTC_OSPEEDR_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)reg_pointer |=  _2BIT_VAL(6, 0x2u);

    /*Configure pulled-up PUPDR[13:12] = 01*/
    reg_pointer = (uint32_t*)PORTC_PUPDR_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)reg_pointer |=  _2BIT_VAL(6, 0x1u);

    // PUPDR[13:12] = 10  (pull-down → idle LOW)
    // *(volatile uint32_t*)PORTC_PUPDR_REGISTER &= ~_2BIT_FIELD(6);
    // *(volatile uint32_t*)PORTC_PUPDR_REGISTER |=  _2BIT_VAL(6, 0x2u);
}


void initGpioB0AsOutput( void )
{
    uint32_t  * reg_pointer;
    /* GPIOB Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOB0 configured as output MODER[1:0] = 01*/
    reg_pointer = (uint32_t*)PORTB_MODER_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_2BIT_FIELD(0);  /* clear bits 1:0   */
    *(volatile uint32_t*)reg_pointer |=  _2BIT_VAL(0, 0x1u);  /* set   bits 1:0=01 */
    
    /*GPIOB0 configured as push-pull OTYPER[0] = 0*/
    reg_pointer = (uint32_t*)PORTB_OTYPER_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_1BIT_FIELD(0);          /* clear bit 0 */

    /*GPIOB0 configured floating PUPDR[1:0] = 00 (no pull)*/
    reg_pointer = (uint32_t*)PORTB_PUPDR_REGISTER;
    *(volatile uint32_t*)reg_pointer &= ~_2BIT_FIELD(0);  /* ensure 00 */
    
    /* GPIOB0 driven high to start out with */
    reg_pointer = (uint32_t*)PORTB_BSRR_REGISTER;
    *(volatile uint32_t*)reg_pointer = _1BIT_FIELD(0);            /* set PB0 high */    
}



void toggleGPIOB0( void )
{
    uint32_t value;
    uint32_t  * reg_pointer;
    //get the current value of the pin 
    value = (*(volatile uint32_t*)PORTB_ODR_REGISTER) & _1BIT_FIELD(0);
    if (value > 0)
    {
        //if high, clear the bit
        *(volatile uint32_t*)PORTB_BSRR_REGISTER = (_1BIT_FIELD(0) << 16);  /* reset PB0 */
    }
    else
    {
        //if low, set the bit
       *(volatile uint32_t*)PORTB_BSRR_REGISTER = _1BIT_FIELD(0);           /* set PB0 */
    } 
}



void setGPIOB0( void )
{
    *(volatile uint32_t*)PORTB_BSRR_REGISTER = _1BIT_FIELD(0);     
}
void clearGPIOB0( void )
{
     *(volatile uint32_t*)PORTB_BSRR_REGISTER = (_1BIT_FIELD(0) << 16);    
}

uint32_t checkGPIOC6(void)
{
    /* Return the sampled logic level on PC6 (0 or 1) */
    uint32_t valueC6 = ((*(volatile uint32_t*)PORTC_IDR_REGISTER) >> 6) & 1u;
    return valueC6;   
}

/////TIMERSssssssss...
void initGpioB0AsAF2_TIM3CH3(void)
{
    /* Turn on GPIOB clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* PB0 -> Alternate Function (MODER[1:0] = 10) */
    *(volatile uint32_t*)PORTB_MODER_REGISTER  &= ~_2BIT_FIELD(0);
    *(volatile uint32_t*)PORTB_MODER_REGISTER  |=  _2BIT_VAL(0, 0x2u);

    /* Push-pull, no pull */
    *(volatile uint32_t*)PORTB_OTYPER_REGISTER &= ~_1BIT_FIELD(0);
    *(volatile uint32_t*)PORTB_PUPDR_REGISTER  &= ~_2BIT_FIELD(0);

    /*We want it Fasssssssst*/
    /* So PB0 : OSPEEDR[1:0] = 10 */
    *(volatile uint32_t*)PORTB_OSPEEDR_REGISTER &= ~_2BIT_FIELD(0);
    *(volatile uint32_t*)PORTB_OSPEEDR_REGISTER |= _2BIT_VAL(0, 0x2u);

    /* AFRL nibble for PB0 = AF2 (TIM3/4/5) */
    /* *4 because it has a 4 bit field */
    *(volatile uint32_t*)PORTB_AFR1_REGISTER   &= ~(0xFu << (0 * 4));
    *(volatile uint32_t*)PORTB_AFR1_REGISTER   |=  (0x2u << (0 * 4));
}

void initGpioC6AsAF2_TIM3CH1(void)
{
    /* Enable GPIOC clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PC6 → Alternate Function (MODER[13:12] = 10) */
    *(volatile uint32_t*)PORTC_MODER_REGISTER &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_MODER_REGISTER |=  _2BIT_VAL(6, 0x2u);

    /* Push-pull (harmless) and no pull (source will drive) */
    *(volatile uint32_t*)PORTC_OTYPER_REGISTER &= ~_1BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER  &= ~_2BIT_FIELD(6);

    /* AFRL nibble for PC6 = AF2 (TIM3 group) → bits [27:24] = 0x2 */
    *(volatile uint32_t*)PORTC_AFRL_REGISTER &= ~(0xFu << (6 * 4));
    *(volatile uint32_t*)PORTC_AFRL_REGISTER |=  (0x2u << (6 * 4));
}

//Helpers
void PC6_use_pullup(void) {
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER |=  _2BIT_VAL(6, 0x1u); // 01 = pull-up
}
void PC6_use_pulldown(void) {
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER |=  _2BIT_VAL(6, 0x2u); // 10 = pull-down
}