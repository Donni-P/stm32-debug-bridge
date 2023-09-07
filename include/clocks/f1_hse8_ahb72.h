#pragma once

#include <stm32f1xx.h>

namespace clock{
static inline void init(void) {
    uint32_t reservedBitsCr =
        RCC->CR & ~(RCC_CR_PLLRDY_Msk | RCC_CR_PLLON_Msk | RCC_CR_CSSON_Msk |
                    RCC_CR_HSEBYP_Msk | RCC_CR_HSERDY_Msk | RCC_CR_HSEON_Msk |
                    RCC_CR_HSIRDY_Msk | RCC_CR_HSION_Msk);
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


    SysTick->CTRL = 0x00;
    SysTick->LOAD = (HSI_VALUE/8)-1;
    SysTick->VAL = 0x00;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    while (((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)&&((RCC->CR & RCC_CR_HSERDY_Msk) == 0))
        ;
    SysTick->CTRL = 0x00;

    if ( (RCC->CR & RCC_CR_HSERDY_Msk) != 0 ){
        FLASH->ACR = FLASH_ACR_PRFTBE | (2<<FLASH_ACR_LATENCY_Pos);
        RCC->CFGR = reservedBitsCfgr | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC |
                    RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE2_DIV1 |
                    RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_SW_HSI;
    }else{
        FLASH->ACR = FLASH_ACR_PRFTBE | (2<<FLASH_ACR_LATENCY_Pos);
        RCC->CFGR = reservedBitsCfgr | RCC_CFGR_USBPRE | RCC_CFGR_PLLMULL12 |
                    RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE2_DIV1 |
                    RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_SW_HSI;
    }

    RCC->CR = reservedBitsCr | RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY_Msk) == 0)
        ;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
        ;
    RCC->APB2ENR = RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR = RCC_APB1ENR_USBEN | RCC_APB1ENR_BKPEN;
    RCC->AHBENR = RCC_AHBENR_FLITFEN | RCC_AHBENR_SRAMEN | RCC_AHBENR_DMA1EN;
    SystemCoreClockUpdate();
}
}

