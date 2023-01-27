
#include <stm32f1xx.h>

#define GLOBAL_RESOURCES_DEFINITIONS
#include <global_resources.h>

#include <gpio.h>
#include <jtag.h>
#include <usb.h>


void ClockInit(void) {
    uint32_t reservedBitsCr =
        RCC->CR & ~(RCC_CR_PLLRDY_Msk | RCC_CR_PLLON_Msk | RCC_CR_CSSON_Msk |
                    RCC_CR_HSEBYP_Msk | RCC_CR_HSERDY_Msk | RCC_CR_HSEON_Msk |
                    RCC_CR_HSITRIM_Msk | RCC_CR_HSIRDY_Msk | RCC_CR_HSION_Msk);
    uint32_t reservedBitsCfgr =
        RCC->CFGR &
        ~(RCC_CFGR_MCO_Msk | RCC_CFGR_USBPRE_Msk | RCC_CFGR_PLLMULL_Msk |
          RCC_CFGR_PLLXTPRE_Msk | RCC_CFGR_PLLSRC_Msk | RCC_CFGR_ADCPRE_Msk |
          RCC_CFGR_PPRE2_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_HPRE_Msk |
          RCC_CFGR_SWS_Msk | RCC_CFGR_SW_Msk);

    RCC->CR = RCC->CR | RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY_Msk) == 0)
        ;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
        ;

    RCC->CR = reservedBitsCr | RCC_CR_HSION | RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_PLLRDY_Msk) != 0)
        ;

    RCC->CFGR = reservedBitsCfgr | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC |
                RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE2_DIV1 |
                RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_SW_HSI;

    FLASH->ACR =
        (FLASH->ACR & ~(FLASH_ACR_HLFCYA_Msk | FLASH_ACR_LATENCY_Msk)) |
        (FLASH_ACR_PRFTBE_Msk | FLASH_ACR_LATENCY_2);

    while ((RCC->CR & RCC_CR_HSERDY_Msk) == 0)
        ;

    RCC->CR = reservedBitsCr | RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY_Msk) != 0)
        ;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
        ;
    RCC->APB1ENR = RCC->APB1ENR | RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN;
    RCC->AHBENR = RCC->AHBENR | RCC_AHBENR_DMA1EN;
    SystemCoreClockUpdate();
}

extern "C" void __terminate() {
    __disable_irq();
    global::led.writeLow();
#ifdef NDEBUG
    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
#endif
    while (1)
        ;
}

static void PortsInit(void) {

    global::led.clockOn();
    global::led.writeHigh();
    global::led.configOutput(gpio::OutputType::gen_pp,
                             gpio::OutputSpeed::_50mhz);

    global::usbPins.clockOn();
    global::usbPins.write(false, false);
    global::usbPins.configOutput<0>(gpio::OutputType::gen_pp,
                                    gpio::OutputSpeed::_50mhz);
    global::usbPins.configOutput<1>(gpio::OutputType::gen_pp,
                                    gpio::OutputSpeed::_50mhz);
    global::jtagOut.clockOn();
    global::jtagOut.write(false, false, false);
    global::jtagOut.configOutput<0>(gpio::OutputType::gen_od,
                                    gpio::OutputSpeed::_50mhz);
    global::jtagOut.configOutput<1>(gpio::OutputType::gen_od,
                                    gpio::OutputSpeed::_50mhz);
    global::jtagOut.configOutput<2>(gpio::OutputType::gen_od,
                                    gpio::OutputSpeed::_50mhz);

    global::jtagIn.clockOn();
    global::jtagIn.write(true);
    global::jtagIn.configInput(gpio::InputType::pull_up_down);

    global::uartPins.clockOn();
    global::uartPins.write(false, true, true, true);
    global::uartPins.configInput<0>(gpio::InputType::floating);  // RX
    global::uartPins.configOutput<1>(gpio::OutputType::alt_pp,
                                     gpio::OutputSpeed::_50mhz); // TX
    global::uartPins.configInput<2>(gpio::InputType::pull_up_down); // CTS
    global::uartPins.configOutput<3>(gpio::OutputType::alt_pp,
                                     gpio::OutputSpeed::_50mhz); // RTS

}

int main() {
    ClockInit();
    PortsInit();
    __enable_irq();

    uint32_t lastDmaRxLen = 0;
    uint32_t lastDmaTxLen = 0;
    uint32_t lastDmaBbTxLen = 0;
    usb::cdcPayload::applyLineCoding();
    {
        auto [addr, len] = global::uartRx.dmaPush();
        DMA1_Channel6->CNDTR = len;
        lastDmaRxLen = len;
        DMA1_Channel6->CMAR = reinterpret_cast<uintptr_t>(addr);
        DMA1_Channel6->CPAR = reinterpret_cast<uintptr_t>(&USART2->DR);
        DMA1_Channel6->CCR = DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_PSIZE_1 |
                             DMA_CCR_CIRC | DMA_CCR_EN;
    }

    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
    TIM2->CR2 = TIM_CR2_CCDS;
    TIM2->SMCR = 0;
    TIM2->DIER = TIM_DIER_UDE;
    TIM2->CCMR1 = 0;
    TIM2->CCMR2 = 0;
    TIM2->CCER = 0;
    TIM2->PSC = 0;
    TIM2->ARR = (SystemCoreClock / 1000000) - 1;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    usb::init();
    while (1) {
        if (usb::cdcPayload::isPendingApply()) {
            usb::cdcPayload::applyLineCoding();
        }
        if (uint32_t dmaRxLen = DMA1_Channel6->CNDTR;
            lastDmaRxLen - dmaRxLen != 0) {
            global::uartRx.dmaPushApply((lastDmaRxLen - dmaRxLen) %
                                        global::uartRx.capacity());
            lastDmaRxLen = dmaRxLen;
        }
        if (uint32_t dmaTxLen = DMA1_Channel7->CNDTR;
            (lastDmaTxLen - dmaTxLen != 0) || (!global::uartTx.empty())) {
            if (dmaTxLen == 0) {
                DMA1_Channel7->CCR = 0;
                global::uartTx.dmaPopApply(lastDmaTxLen);
                auto [addr, len] = global::uartTx.dmaPop();
                DMA1_Channel7->CNDTR = len;
                lastDmaTxLen = len;
                DMA1_Channel7->CMAR = reinterpret_cast<uintptr_t>(addr);
                DMA1_Channel7->CPAR = reinterpret_cast<uintptr_t>(&USART2->DR);
                DMA1_Channel7->CCR =
                    DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
            } else {
                global::uartTx.dmaPopApply(lastDmaTxLen - dmaTxLen);
                lastDmaTxLen = dmaTxLen;
            }
        }

        if (uint32_t dmaTxLen = DMA1_Channel2->CNDTR;
            (lastDmaBbTxLen - dmaTxLen != 0) || (!global::bbTx.empty())) {
            if (dmaTxLen == 0) {
                DMA1_Channel2->CCR = 0;
                global::bbTx.dmaPopApply(lastDmaBbTxLen);
                auto [addr, len] = global::bbTx.dmaPop();
                DMA1_Channel2->CNDTR = len;
                lastDmaBbTxLen = len;
                DMA1_Channel2->CMAR = reinterpret_cast<uintptr_t>(addr);
                DMA1_Channel2->CPAR = reinterpret_cast<uintptr_t>(
                    &(global::jtagOut.getGpioPointer()->BSRR));
                DMA1_Channel2->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 |
                                     DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
            } else {
                global::bbTx.dmaPopApply(lastDmaBbTxLen - dmaTxLen);
                lastDmaBbTxLen = dmaTxLen;
            }
        }
        usb::regenerateTx();
        jtag::tick();
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
