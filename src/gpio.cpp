#include <gpio.h>


namespace gpio{
    
__attribute__((section(".gpioa"))) volatile GpioRegs gpioa[1];
__attribute__((section(".gpiob"))) volatile GpioRegs gpiob[1];
__attribute__((section(".gpioc"))) volatile GpioRegs gpioc[1];
__attribute__((section(".gpiod"))) volatile GpioRegs gpiod[1];
__attribute__((section(".gpioe"))) volatile GpioRegs gpioe[1];
__attribute__((section(".gpiof"))) volatile GpioRegs gpiof[1];
__attribute__((section(".gpiog"))) volatile GpioRegs gpiog[1];

__attribute__((section(".afio"))) volatile Afio::Regs Afio::data[1];

}
