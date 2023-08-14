set(STARTUP src/startup-stm32f103.cpp)

if(NOT STM32CUBE)
set(STM32CUBE ${PROJECT_SOURCE_DIR}/STM32CubeF1)
endif(NOT STM32CUBE)

include_directories(include
    ${STM32CUBE}/Drivers/CMSIS/Core/Include
    ${STM32CUBE}/Drivers/CMSIS/Core_A/Include
    ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${PLATFORM_DIR}
)

set (STM32_LDSCRIPT ${PROJECT_SOURCE_DIR}/src/STM32F103XB_FLASH.ld)

add_compile_definitions(
    PLATFORM_HEADER=<stm32f1xx.h>
    VENDOR_ID=0x1209
    PRODUCT_ID=0xfffe
    VENDOR_STRING=u"RC Module"_sd
    PRODUCT_STRING=u"malina debug bridge"_sd
    CDC_FIFO_LEN_RX=1024
    CDC_FIFO_LEN_TX=1024
    GPIO_USB_PORT=gpio::Port::a
    GPIO_USB_P=12
    GPIO_USB_N=11
    GPIO_JTAG_OUT_PORT=gpio::Port::b
    GPIO_JTAG_TDI=15
    GPIO_JTAG_TMS=12
    GPIO_JTAG_TCK=13
    GPIO_JTAG_IN_PORT=gpio::Port::b
    GPIO_JTAG_TDO=14
    GPIO_UART_PORT=gpio::Port::a
    GPIO_UART_RX=10
    GPIO_UART_TX=9
    UART_CLK_DIV=1
    UART=USART1
    UART_DMA_TX=DMA1_Channel4
    UART_DMA_RX=DMA1_Channel5
)
set (MCPU cortex-m3)
