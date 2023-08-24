
#include PLATFORM_HEADER

#include <global_resources.h>

#include <gpio.h>
#include <jtag.h>
#include <usb.h>
#include <shell.h>

#include CLOCK_INIT_HEADER
#include COMMANDS_HEADER
uint32_t results[6] = {0};
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

    static uint32_t wordDma[6]={config::portPins.led.makeWriteWord(false),
                                config::portPins.led.makeWriteWord(true),
                                config::portPins.led.makeWriteWord(false),
                                config::portPins.led.makeWriteWord(true),
                                config::portPins.led.makeWriteWord(false),
                                config::portPins.led.makeWriteWord(true)};

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
    const static char prompt[] = "> ";
    shell::Shell<
        commands::CommandExecutor,
        prompt,
        60,
        8,
        shell::color::index::green,
        false,
        false,
        false,
        16
    > sh;
    config::configInit();
    usb::init();
    __enable_irq();
    DMA1_Channel2->CNDTR = 0x6;
    DMA1_Channel2->CPAR = (uint32_t)&config::portPins.led.getGpioPointer()->BSRR;
    DMA1_Channel2->CMAR = (uint32_t)wordDma;
    DMA1_Channel3->CNDTR = 0x6;
    DMA1_Channel3->CPAR = (uint32_t)&config::portPins.led.getGpioPointer()->IDR;
    DMA1_Channel3->CMAR = (uint32_t)results;

    RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_TIM1EN;
    TIM1->DIER = TIM1->DIER | TIM_DIER_UIE | TIM_DIER_CC1DE | TIM_DIER_CC2DE;
    TIM1->PSC = 0xf9ff; //72 МГц / 64 кГц = 1125 Гц
    TIM1->ARR = 0x2be8; // 10 секунд
    TIM1->CCR1 = 0x15f4;//50% скважность - 5 секунд
    TIM1->CCR2 = 0x15f4;
    TIM1->RCR = 0x5; // 1 минута 
    TIM1->CCMR1 = TIM1->CCMR1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM1->CR1 = TIM1->CR1 | TIM_CR1_URS | TIM_CR1_CEN;
    TIM1->EGR = TIM1->EGR | TIM_EGR_UG;
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    DMA1_Channel2->CCR = DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_MINC | 
                         DMA_CCR_DIR | DMA_CCR_EN;
    DMA1_Channel3->CCR = DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_MINC | DMA_CCR_EN;                     
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
            sh.exec(global::shellTx.pop());
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
extern "C" void TIM1_UP_IRQHandler(){
    if((TIM1->SR & TIM_SR_UIF_Msk) != 0){
        //config::portPins.led.write( ! config::portPins.led.read());
        TIM1->SR = TIM1->SR & ~TIM_SR_UIF_Msk;
    }
    //TIM1->CR1 = TIM1->CR1 & ~TIM_CR1_CEN_Msk;
}