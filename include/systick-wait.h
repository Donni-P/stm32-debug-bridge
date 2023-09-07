#include PLATFORM_HEADER

template <typename Condition,typename Body>
void systickWait(uint16_t us, const Condition &condition, const Body &body){
    uint64_t us_ = us;
    us_ *= 16000000;
    us_ /= SystemCoreClock;
    us_ = us_/2 + us_%2;
    if ( us_ >= 0x1000000u ){
        us_ = 0x1000000u;
    }
    us_--;

    SysTick->CTRL = 0x00;
    SysTick->LOAD = us_;
    SysTick->VAL = 0x00;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    while (((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)&&condition())
        body();
    SysTick->CTRL = 0x00;
}
