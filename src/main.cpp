
#include PLATFORM_HEADER

#include <global_resources.h>

#include <gpio.h>
#include <jtag.h>
#include <usb.h>
#include <device-shell.h>

#include CLOCK_INIT_HEADER

static inline void PortsInit(void) {

    using namespace gpio;

    global::usbPins.clockOn();
    global::usbPins.configInput<0>(gpio::InputType::floating);
    global::usbPins.configInput<1>(gpio::InputType::floating);
    global::usbPins.write(false, false);

    global::jtagOut.clockOn();
    global::jtagOut.write(false, false, false);
    global::jtagOut.configOutput<0>(OutputType::gen_pp,
                                    OutputSpeed::_10mhz);
    global::jtagOut.configOutput<1>(OutputType::gen_pp,
                                    OutputSpeed::_10mhz);
    global::jtagOut.configOutput<2>(OutputType::gen_pp,
                                    OutputSpeed::_10mhz);

    global::jtagIn.clockOn();
    global::jtagIn.write(true);
    global::jtagIn.configInput(InputType::pull_up_down);

    global::uartPins.clockOn();
    global::uartPins.write<0>(false);
    global::uartPins.write<1>(true);
    global::uartPins.configInput<0>(InputType::floating);  // RX
    global::uartPins.configOutput<1>(OutputType::alt_pp,
                                     OutputSpeed::_10mhz); // TX

    config::PortsInit();
}

extern "C" void __terminate() {
    __disable_irq();
    config::Panic();
#ifdef NDEBUG
    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
#endif
    while (1)
        ;
}

int main() {
    clock::init();
    SystemCoreClockUpdate();
    PortsInit();
    deviceShell::tick('\b');

    uint32_t lastDmaRxLen = 0;
    uint32_t lastDmaTxLen = 0;
    usb::cdcPayload::applyLineCoding();
    {
        auto [addr, len] = global::uartRx.dmaPush();
        global::uartDmaRx->CNDTR = len;
        lastDmaRxLen = len;
        global::uartDmaRx->CMAR = reinterpret_cast<uintptr_t>(addr);
        global::uartDmaRx->CPAR = reinterpret_cast<uintptr_t>(&global::uart->DR);
        global::uartDmaRx->CCR = DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_PSIZE_1 |
                             DMA_CCR_CIRC | DMA_CCR_EN;
    }

    config::configInit();

    usb::init();
    __enable_irq();
    while (1) {
        if (usb::cdcPayload::isPendingApply()) {
            usb::cdcPayload::applyLineCoding();
        }
        if (uint32_t dmaRxLen = global::uartDmaRx->CNDTR;
            lastDmaRxLen - dmaRxLen != 0) {
            global::uartRx.dmaPushApply((lastDmaRxLen - dmaRxLen) %
                                        global::uartRx.capacity());
            lastDmaRxLen = dmaRxLen;
        }
        if (uint32_t dmaTxLen = global::uartDmaTx->CNDTR;
            (lastDmaTxLen - dmaTxLen != 0) || (!global::uartTx.empty())) {
            if (dmaTxLen == 0) {
                global::uartDmaTx->CCR = 0;
                global::uartTx.dmaPopApply(lastDmaTxLen);
                auto [addr, len] = global::uartTx.dmaPop();
                global::uartDmaTx->CNDTR = len;
                lastDmaTxLen = len;
                global::uartDmaTx->CMAR = reinterpret_cast<uintptr_t>(addr);
                global::uartDmaTx->CPAR = reinterpret_cast<uintptr_t>(&global::uart->DR);
                global::uartDmaTx->CCR =
                    DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
            } else {
                global::uartTx.dmaPopApply(lastDmaTxLen - dmaTxLen);
                lastDmaTxLen = dmaTxLen;
            }
        }

        usb::regenerateTx();
        while( !global::shellTx.empty() )
        {
            deviceShell::tick(global::shellTx.pop());
        }
        while ( !global::jtagTx.empty() )
        {
            jtag::tick(global::jtagTx.pop());
        }
        if (!global::uartRx.empty()) {
            usb::sendFromFifo(usb::descriptor::InterfaceIndex::uart,
                              global::uartRx);
        }
        if (!global::shellRx.empty()) {
            usb::sendFromFifo(usb::descriptor::InterfaceIndex::shell,
                              global::shellRx);
        }
        if (!global::jtagRx.empty()) {
            usb::sendFromFifo(usb::descriptor::InterfaceIndex::jtag,
                              global::jtagRx);
        }
    }

    return 0;
}
