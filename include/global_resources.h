#pragma once

#include <cdc_payload.h>
#include <fifo.h>
#include <gpio.h>


namespace global {

constexpr std::size_t cdcFifoLenRx = CDC_FIFO_LEN_RX;
constexpr std::size_t cdcFifoLenTx = CDC_FIFO_LEN_TX;

using UartRxType = fifo::Fifo<std::uint8_t, cdcFifoLenRx>;
using UartTxType = fifo::Fifo<std::uint8_t, cdcFifoLenTx>;

using ShellRxType = fifo::Fifo<std::uint8_t, cdcFifoLenRx>;
using ShellTxType = fifo::Fifo<std::uint8_t, cdcFifoLenTx>;

using JtagRxType = fifo::Fifo<std::uint8_t, cdcFifoLenRx>;
using JtagTxType = fifo::Fifo<std::uint8_t, cdcFifoLenTx>;

using UsbPinsType = gpio::Bulk<
    GPIO_USB_PORT,
    GPIO_USB_N,
    GPIO_USB_P
>;

using JtagOutType = gpio::Bulk<
    GPIO_JTAG_OUT_PORT,
    GPIO_JTAG_TDI,
    GPIO_JTAG_TMS,
    GPIO_JTAG_TCK
>;
using JtagInType = gpio::Pin<
    GPIO_JTAG_IN_PORT,
    GPIO_JTAG_TDO
>;

using UartPinsType = gpio::Bulk<
    GPIO_UART_PORT,
    GPIO_UART_RX,
    GPIO_UART_TX
>;

extern UartPinsType uartPins;
constexpr uint32_t uartClkDiv = UART_CLK_DIV;

extern uint16_t flashconfig[512];

struct LineCodingControl : usb::cdcPayload::LineCoding {
    bool isChanged;
};

extern UartRxType uartRx;
extern UartTxType uartTx;
extern ShellRxType shellRx;
extern ShellTxType shellTx;
extern JtagRxType jtagRx;
extern JtagTxType jtagTx;
extern LineCodingControl uartLineCoding;

extern UsbPinsType usbPins;
extern JtagOutType jtagOut;
extern JtagInType jtagIn;

static USART_TypeDef * const uart = UART;
static DMA_Channel_TypeDef * const uartDmaTx = UART_DMA_TX;
static DMA_Channel_TypeDef * const uartDmaRx = UART_DMA_RX;

} // namespace global

#include <config.h>

