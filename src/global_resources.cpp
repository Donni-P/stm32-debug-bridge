#include <global_resources.h>

namespace global {
UartRxType uartRx;
UartTxType uartTx;
ShellRxType shellRx;
ShellTxType shellTx;
JtagRxType jtagRx;
JtagTxType jtagTx;
LineCodingControl uartLineCoding;
UartPinsType uartPins;
UsbPinsType usbPins;
JtagOutType jtagOut;
JtagInType jtagIn;
__attribute__ ((section(".flashconfig"))) uint16_t flashconfig[512];
UsbPinsType usbPins;
JtagOutType jtagOut;
JtagInType jtagIn;
}

namespace config {
PortPins portPins;
}

