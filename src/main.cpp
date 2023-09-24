
#include PLATFORM_HEADER

#include <global_resources.h>

#include <gpio.h>
#include <jtag.h>
#include <usb.h>
#include <device-shell.h>

#include CLOCK_INIT_HEADER

#include <systick-wait.h>

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

template<typename uint_T,int amount_regs>
class Hal{
public:
    static void clear(){
        for(int i = 0; i < amount_regs; i++)
            mem[i] = 0x0;
    }
    static void rx(uint_T * mm){
        clear();
        for(int i = 0; i < amount_regs; i++){
            mem[i] = mm[i];
        }
    }
    static uint_T tx(){
        return mem;
    }
    static inline uint_T mem[amount_regs] = {0x0};
};
template<typename uint_T, int amount_regs, class HAL1, class HAL2>
class SlaveI2C {
public:
    enum size{
        bits_8 = 1,
        bits_16,
        bits_32 = 4
    };
    enum state{
        stay,//нет передачи, ждется START
        busy,//состояние, в котором после каждого байта проверяется есть ли START или STOP
        receive,//запись байта в регистр
        transmit,//передача байта в мастер

    };
    void reset(){
        for(int i = 0; i < amount_regs; i++)
            mem[i] = 0x0;
        ind_reg = 0;
    }
    void send_toHAL(){
        if(!dualf)
            HAL1::rx(mem);
        else HAL2::rx(mem);
        reset();
    }
    void rx_fromHAL(){
        uint_T * mm;
        if(!dualf)
            mm = HAL1::tx();
        else mm = HAL2::tx();
        for(int i = 0; i < amount_regs; i++){
            mem[i] = mm[i];
        }
    }
    void read_fromMaster(size nbytes){
        if(ind_reg < (amount_regs * nbytes)){
            mem[ind_reg / nbytes] |= (i2c_crl->DR << (8 * (ind_reg % nbytes)));
            ind_reg++;
        }
    }
    void write_toMaster(size nbytes){
        uint8_t byte;
        if(ind_reg < (amount_regs * nbytes)){
            byte = mem[ind_reg / nbytes] >> (8 * (ind_reg % nbytes));
            i2c_crl->DR = byte; 
        }
    }
    void set_modeHAL() {
        if((i2c_crl->SR2 & I2C_SR2_TRA_Msk) == 0){
            slv_state = receive;
            tra = false;
        }else {
            slv_state = transmit;
            tra = true;
        }
        if((i2c_crl->SR2 & I2C_SR2_DUALF_Msk) == 0)
            dualf = false;
        else dualf = true;
    }
    bool addr_rx(){
        bool res = false;
        if((i2c_crl->SR1 & I2C_SR1_ADDR_Msk) != 0){
            i2c_crl->SR1 = i2c_crl->SR1;
            i2c_crl->SR2 = i2c_crl->SR2;
            set_modeHAL();
            res = true;
        }
        return res;
    }
    void check(){ //точка входа
        switch(slv_state){
            case stay:
                addr_rx();
                break;
            case busy:
                if(addr_rx()){
                    if(!tra) send_toHAL();
                }else if((i2c_crl->SR1 & I2C_SR1_STOPF_Msk) != 0){
                    i2c_crl->SR1 = i2c_crl->SR1;
                    i2c_crl->CR1 = i2c_crl->CR1;
                    send_toHAL();
                    slv_state = stay;
                }else if((i2c_crl->SR1 & I2C_SR1_AF_Msk) != 0){
                    i2c_crl->SR1 = i2c_crl->SR1 & ~I2C_SR1_AF_Msk;
                    reset();
                }else set_modeHAL();
                break;
            case receive:
                //получение
                if((i2c_crl->SR1 & I2C_SR1_RXNE_Msk) != 0){
                    switch(slv_size){
                        case bits_8:
                            read_fromMaster(bits_8);
                            break;
                        case bits_16:
                            read_fromMaster(bits_16);
                            break;
                        case bits_32:
                            read_fromMaster(bits_32);
                            break;
                            
                    }
                    slv_state = busy;
                }
                break;
            case transmit:
                //передача
                if((i2c_crl->SR1 & I2C_SR1_TXE_Msk) != 0){
                    switch(slv_size){
                        case bits_8:
                            write_toMaster(bits_8);
                            break;
                        case bits_16:
                            write_toMaster(bits_16);
                            break;
                        case bits_32:
                            write_toMaster(bits_32);
                            break;
                            
                    }
                    slv_state = busy;
                }
                break;
        }
    }
    SlaveI2C(I2C_TypeDef *i2c):slv_state(stay),ind_reg(0),i2c_crl(i2c){
        slv_size = static_cast<size>(sizeof(uint_T));
    }

private:
    state slv_state;
    size slv_size;
    int ind_reg;
    uint_T mem[amount_regs] = {0x0};
    I2C_TypeDef * const i2c_crl;
    bool tra = false;
    bool dualf = false;
};


int main() {
    clock::init();
    SystemCoreClockUpdate();
    PortsInit();

    SlaveI2C<uint32_t, 4, Hal<uint32_t,4>,Hal<uint32_t,4>> x(I2C2);

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
    usb::cdcPayload::applyLineCoding();
    deviceShell::tick('\b');

    usb::init();
    __enable_irq();
    RCC->APB1ENR = RCC->APB1ENR | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN;
    RCC->APB2ENR = (RCC->APB2ENR & ~RCC_APB2ENR_IOPBEN_Msk) | RCC_APB2ENR_IOPBEN;
    //I2C1
    GPIOB->CRL = (GPIOB->CRL & ~(0xff << 24)) | (0xff << 24);//alternate open-drain
    GPIOB->ODR = (GPIOB->ODR & ~(0x3 << 6)) | (0x3 << 6);// 11 в 7 и 6 битах ODR
    //I2C2
    GPIOB->CRH = (GPIOB->CRH & ~(0xff << 8)) | (0xff << 8);//alternate open-drain
    GPIOB->ODR = (GPIOB->ODR & ~(0x3 << 10)) | (0x3 << 10);// 11 в 7 и 6 битах ODR
    //I2C1
    I2C1->CR1 = I2C1->CR1 | I2C_CR1_SWRST;
    while((I2C1->SR2 & I2C_SR2_BUSY_Msk) != 0){}
    I2C1->CR1 = I2C1->CR1 & ~I2C_CR1_SWRST_Msk;
    //I2C1->OAR1 = (0x11 << 1);
    I2C1->CR2 = 0x24; // 36 MHz
    I2C1->CCR = 0xb4; // 100 KHz
    I2C1->TRISE = 0x25; // 37
    I2C1->CR1 = I2C1->CR1 | I2C_CR1_ACK | I2C_CR1_PE;
    //I2C2
    I2C2->CR1 = I2C2->CR1 | I2C_CR1_SWRST;
    while((I2C2->SR2 & I2C_SR2_BUSY_Msk) != 0){}
    I2C2->CR1 = I2C2->CR1 & ~I2C_CR1_SWRST_Msk;
    I2C2->OAR1 = (0x33 << 1);
    I2C2->OAR2 = (0x42 << 1) | 1;
    I2C2->CR2 = 0x24; // 36 MHz
    I2C2->CCR = 0xb4; // 100 KHz
    I2C2->TRISE = 0x25; // 37
    I2C2->CR1 = I2C2->CR1 | I2C_CR1_ACK | I2C_CR1_PE;
    //transmit
    //packet 1
    I2C1->CR1 = I2C1->CR1 | I2C_CR1_START;
    while((I2C1->SR1 & I2C_SR1_SB_Msk) == 0){}
    I2C1->SR1 = I2C1->SR1;//read
    I2C1->DR = (0x33 << 1); 
    while ((I2C1->SR1 & I2C_SR1_ADDR_Msk) == 0){
        x.check();
    }
    I2C1->SR1 = I2C1->SR1; //read
    I2C1->SR2 = I2C1->SR2; //read
    I2C1->DR = 0x38; 
    while ((I2C1->SR1 & I2C_SR1_TXE_Msk) == 0){
        x.check();
    }
    I2C1->DR = 0x11; 
    while ((I2C1->SR1 & I2C_SR1_TXE_Msk) == 0){
        x.check();
    }
    I2C1->CR1 = I2C1->CR1 | I2C_CR1_STOP;
    while((I2C2->SR1 & I2C_SR1_STOPF_Msk) == 0){
        x.check();
    }

    RCC->CSR = RCC_CSR_LSION;
    PWR->CR = PWR->CR | PWR_CR_DBP;
    NVIC_EnableIRQ(RCC_IRQn);
    RCC->CIR = RCC_CIR_LSIRDYIE | RCC_CIR_LSERDYIE;
    RCC->BDCR = RCC_BDCR_RTCEN | RCC_BDCR_LSEON;
    RCC->CIR = RCC_CIR_LSIRDYIE;
    while (1) {
        if (usb::cdcPayload::isPendingApply()) {
            usb::cdcPayload::applyLineCoding();
        }
        if (uint32_t dmaRxLen = global::uartDmaRx->CNDTR;
            lastDmaRxLen - dmaRxLen != 0) {
            global::uartRx.dmaPushApply((lastDmaRxLen - dmaRxLen) %
                                        global::uartRx.capacity());
            lastDmaRxLen = dmaRxLen;
            __disable_irq();
            if ( global::uartRx.size() > global::uartRx.capacity() ){
                global::uartRx.dmaPopApply(global::uartRx.size() - global::uartRx.capacity());
            }
            __enable_irq();
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

extern "C" void RCC_IRQHandler(){
    uint32_t rtcPreDiv = 0;
    uint32_t zeroClock = 0;
    if ( RCC->CIR & RCC_CIR_LSIRDYF ){
        if ((RCC->BDCR&RCC_BDCR_RTCSEL_Msk)==0){
            RCC->BDCR = RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSI | RCC_BDCR_LSEON;
            rtcPreDiv = 39999u;
            zeroClock = 1693464085;
        }
        RCC->CIR = (RCC->CIR & (~RCC_CIR_LSIRDYIE)) | RCC_CIR_LSIRDYC;
    }
    if ( RCC->CIR & RCC_CIR_LSERDYF ){
        if ((RCC->BDCR&RCC_BDCR_RTCSEL_Msk)==0){
            zeroClock = 1693464085;
        }
        RCC->BDCR = RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE | RCC_BDCR_LSEON;
        RCC->CIR = (RCC->CIR & (~RCC_CIR_LSERDYIE)) | RCC_CIR_LSERDYC;
        rtcPreDiv = 0x7fffu;
    }
    if ( rtcPreDiv ){
        while ((RTC->CRL&RTC_CRL_RTOFF)==0);
        RTC->CRL = RTC_CRL_CNF;
        RTC->PRLH = (rtcPreDiv >> 16);
        RTC->PRLL = (rtcPreDiv & 0xffffu);
        if ( zeroClock ){
            RTC->CNTH = (zeroClock >> 16);
            RTC->CNTL = (zeroClock & 0xffffu);
        }
        RTC->CRL = 0;
        while ((RTC->CRL&RTC_CRL_CNF)!=0);
    }
}
