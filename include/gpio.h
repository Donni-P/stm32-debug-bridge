#pragma once

#include PLATFORM_HEADER

#include <tuple>

#ifndef NDEBUG
#include <stdexcept>
#endif

namespace gpio {

struct GpioRegs{
    uint32_t CRL;
    uint32_t CRH;
    uint32_t IDR;
    uint32_t ODR;
    uint32_t BSRR;
    uint32_t BRR;
    uint32_t LCKR;
};

extern volatile GpioRegs gpioa[1];
extern volatile GpioRegs gpiob[1];
extern volatile GpioRegs gpioc[1];
extern volatile GpioRegs gpiod[1];
extern volatile GpioRegs gpioe[1];
extern volatile GpioRegs gpiof[1];
extern volatile GpioRegs gpiog[1];

enum class Port {
    a,
    b,
    c,
    d,
    e,
    f,
    g,
    end
};

enum class OutputSpeed { _10mhz = 0x1, _2mhz = 0x2, _50mhz = 0x3 };

enum class InputType { analog = 0x0, floating = 0x1, pull_up_down = 0x2 };

enum class OutputType {
    gen_pp = 0x0,
    gen_od = 0x1,
    alt_pp = 0x2,
    alt_od = 0x3
};

template <Port port, uint8_t... pins> class Bulk;

template <Port port, uint8_t pin>
    requires (port < Port::end && pin < 16)
class Pin {

  private:
    static uint32_t getClockMsk() {
        uint32_t result;

        switch (port) {
        case Port::a:
            result = 0x1<<(2+0);
            break;
        case Port::b:
            result = 0x1<<(2+1);
            break;
        case Port::c:
            result = 0x1<<(2+2);
            break;
        case Port::d:
            result = 0x1<<(2+3);
            break;
        case Port::e:
            result = 0x1<<(2+4);
            break;
        case Port::f:
            result = 0x1<<(2+5);
            break;
        case Port::g:
            result = 0x1<<(2+6);
            break;
        }
        return result;
    }
#ifndef NDEBUG
    static bool bisy;
#endif
    uint32_t makeWriteWordHigh() const { return 1u << (pin + 0); }
    uint32_t makeWriteWordLow() const { return 1u << (pin + 16); }
    void writeRaw(uint32_t v) {
        getGpioPointer()->BSRR = v;
        __DMB();
    }

  public:
    volatile GpioRegs *getGpioPointer() const {
        volatile GpioRegs *p;
        switch (port) {
        case Port::a:
            p = gpioa;
            break;
        case Port::b:
            p = gpiob;
            break;
        case Port::c:
            p = gpioc;
            break;
        case Port::d:
            p = gpiod;
            break;
        case Port::e:
            p = gpioe;
            break;
        case Port::f:
            p = gpiof;
            break;
        case Port::g:
            p = gpiog;
            break;
        }
        return p;
    }
    uint32_t makeWriteWord(bool v) const {
        if (v) {
            return makeWriteWordHigh();
        }
        return makeWriteWordLow();
    }
    void configInput(InputType type) {
        volatile GpioRegs *p = getGpioPointer();
        if (pin < 8) {
            auto t = p->CRL;
            t &= ~(0xfu << ((pin - 0) * 4));
            t |= static_cast<uint32_t>(type) << ((pin - 0) * 4 + 2);
            p->CRL = t;
        } else {
            auto t = p->CRH;
            t &= ~(0xfu << ((pin - 8) * 4));
            t |= static_cast<uint32_t>(type) << ((pin - 8) * 4 + 2);
            p->CRH = t;
        }
        __DMB();
    }
    void configOutput(OutputType type, OutputSpeed speed) {
        volatile GpioRegs *p = getGpioPointer();
        if (pin < 8) {
            auto t = p->CRL;
            t &= ~(0xfu << ((pin - 0) * 4));
            t |= (static_cast<uint32_t>(speed) |
                  (static_cast<uint32_t>(type) << 2))
                 << ((pin - 0) * 4);
            p->CRL = t;
        } else {
            auto t = p->CRH;
            t &= ~(0xfu << ((pin - 8) * 4));
            t |= (static_cast<uint32_t>(speed) |
                  (static_cast<uint32_t>(type) << 2))
                 << ((pin - 8) * 4);
            p->CRH = t;
        }
        __DMB();
    }
    bool read() const {
        return static_cast<bool>(getGpioPointer()->IDR & ( 1u << pin ) );
    }
    void writeHigh() { writeRaw(makeWriteWordHigh()); }
    void writeLow() { writeRaw(makeWriteWordLow()); }
    void write(bool v) {
        if (v) {
            writeHigh();
        } else {
            writeLow();
        }
    }
    void operator=(bool v) { write(v); }
    static void clockOn() { RCC->APB2ENR = RCC->APB2ENR | getClockMsk(); }
    static void clockOff() { RCC->APB2ENR = RCC->APB2ENR & ~getClockMsk(); }
    void toggle() { write(!read()); }
    Pin([[maybe_unused]] bool unique=true) {
#ifndef NDEBUG
        if(unique){
            auto primask = __get_PRIMASK();
            __disable_irq();
            if (bisy) {
                throw std::bad_exception();
            }
            bisy = true;
            __set_PRIMASK(primask);
        }
#endif
    }
    Pin(const Pin &) = delete;
    Pin(Pin &&) {}
    ~Pin() {
        configInput(InputType::floating);
#ifndef NDEBUG
        auto primask = __get_PRIMASK();
        __disable_irq();
        bisy = false;
        __set_PRIMASK(primask);
#endif
    }
    template <Port p, uint8_t... pins> friend class Bulk;
};

#ifndef NDEBUG
template <Port port, uint8_t pin> bool Pin<port, pin>::bisy = false;
#endif

template <Port port, uint8_t... pins> class Bulk {
  private:
    static constexpr uint8_t ind[sizeof...(pins)] = {pins...};
    std::tuple<Pin<port, pins>...> pins_;
    template <size_t I = 0>
    uint32_t makeWriteWord([[maybe_unused]] const bool v[sizeof...(pins)]) const
        requires(I == sizeof...(pins)) {
        return 0;
    }
    template <size_t I = 0>
    uint32_t makeWriteWord(const bool v[sizeof...(pins)]) const 
        requires(I<sizeof...(pins)) {
        return makeWriteWord<I + 1>(v) | std::get<I>(pins_).makeWriteWord(v[I]);
    }

  public:
    volatile GpioRegs *getGpioPointer() {
        return std::get<0>(pins_).getGpioPointer();
    }
    template <typename... in>
    uint32_t makeWriteWord(in... args) requires(sizeof...(in) ==
                                                sizeof...(pins)) {
        bool v[] = {args...};
        return makeWriteWord(v);
    }
    void clockOn() { std::get<0>(pins_).clockOn(); }
    template <uint8_t I> void configInput(InputType type) {
        std::get<I>(pins_).configInput(type);
    }
    template <uint8_t I> void configOutput(OutputType type, OutputSpeed speed) {
        std::get<I>(pins_).configOutput(type, speed);
    }

    template <typename... in> void write(in... args)
        requires(sizeof...(in) == sizeof...(pins)) {
        bool v[] = {args...};
        std::get<0>(pins_).writeRaw(makeWriteWord(v));
    }
    template <uint8_t I> void write(bool v) { std::get<I>(pins_).write(v); }
    template <uint8_t I> uint32_t makeWriteWord(bool v) {
        return std::get<I>(pins_).makeWriteWord(v);
    }
    template <uint8_t I> bool read() { return std::get<I>(pins_).read(); }
    void writeRaw(uint32_t v){
        std::get<0>(pins_).writeRaw(v);
    }
    Bulk(){}
    template <typename... Args>
        requires(sizeof...(Args) == sizeof...(pins))
    Bulk(Args... args):pins_(args...) {}
};

class Afio{
public:
    struct Regs{
        uint32_t EVCR;
        uint32_t MAPR;
        uint32_t EXTICR1;
        uint32_t EXTICR2;
        uint32_t EXTICR3;
        uint32_t EXTICR4;
        uint32_t MAPR2;
    };
    volatile Regs * operator -> () {return data;}
    struct EVCR{
        static constexpr uint32_t evoeInd = 7;
        static constexpr uint32_t evoeMsk = 0x1u << evoeInd;
        static constexpr uint32_t portInd = 4;
        static constexpr uint32_t portMsk = 0x7u << portInd;
        static constexpr uint32_t pinInd = 0;
        static constexpr uint32_t pinMsk = 0xfu << pinInd;
    };
    struct MAPR{
        static constexpr uint32_t swjCfgInd = 24;
        static constexpr uint32_t swjCfgMsk = 0x7u<<swjCfgInd;
        static constexpr uint32_t swjCfgFull = 0x0u<<swjCfgInd;
        static constexpr uint32_t swjCfgNoNjtrst = 0x1u<<swjCfgInd;
        static constexpr uint32_t swjCfgSwdOnly = 0x2u<<swjCfgInd;
        static constexpr uint32_t swjCfgNone = 0x4u<<swjCfgInd;
        static constexpr uint32_t swjCfgNoEffect = 0x7u<<swjCfgInd;

        static constexpr uint32_t adc2EtrgRegInd = 20;
        static constexpr uint32_t adc2EtrgRegMsk = 1u << adc2EtrgRegInd;
        static constexpr uint32_t adc2EtrgInjInd = 19;
        static constexpr uint32_t adc2EtrgInjMsk = 1u << adc2EtrgRegInd;

        static constexpr uint32_t adc1EtrgRegInd = 18;
        static constexpr uint32_t adc1EtrgRegMsk = 1u << adc2EtrgRegInd;
        static constexpr uint32_t adc1EtrgInjInd = 17;
        static constexpr uint32_t adc1EtrgInjMsk = 1u << adc2EtrgRegInd;

        static constexpr uint32_t tim5Ch4iRemapInd = 16;
        static constexpr uint32_t tim5Ch4iRemapMsk = 1u << tim5Ch4iRemapInd;

        static constexpr uint32_t pd01RemapInd = 15;
        static constexpr uint32_t pd01RemapMsk = 1u << pd01RemapInd;

        static constexpr uint32_t canRemapInd = 13;
        static constexpr uint32_t canRemapMsk = 3u << canRemapInd;
        static constexpr uint32_t canRemapA11A12 = 0u << canRemapInd;
        static constexpr uint32_t canRemapB8B9 = 2u << canRemapInd;
        static constexpr uint32_t canRemapD0D1 = 3u << canRemapInd;

        static constexpr uint32_t tim4RemapInd = 12;
        static constexpr uint32_t tim4RemapMsk = 1u << tim4RemapInd;

        static constexpr uint32_t tim3RemapInd = 10;
        static constexpr uint32_t tim3RemapMsk = 3u << tim3RemapInd;
        static constexpr uint32_t tim3RemapNo = 0u << tim3RemapInd;
        static constexpr uint32_t tim3RemapPartical = 2u << tim3RemapInd;
        static constexpr uint32_t tim3RemapFull = 3u << tim3RemapInd;

        static constexpr uint32_t tim2RemapInd = 8;
        static constexpr uint32_t tim2RemapMsk = 3u << tim2RemapInd;
        static constexpr uint32_t tim2RemapNo = 0u << tim2RemapInd;
        static constexpr uint32_t tim2RemapPart1 = 1u << tim2RemapInd;
        static constexpr uint32_t tim2RemapPart2 = 2u << tim2RemapInd;
        static constexpr uint32_t tim2RemapFull = 3u << tim2RemapInd;

        static constexpr uint32_t tim1RemapInd = 6;
        static constexpr uint32_t tim1RemapMsk = 3u << tim1RemapInd;
        static constexpr uint32_t tim1RemapNo = 0u << tim1RemapInd;
        static constexpr uint32_t tim1RemapPartical = 1u << tim1RemapInd;
        static constexpr uint32_t tim1RemapFull = 3u << tim1RemapInd;

        static constexpr uint32_t usart3RemapInd = 4;
        static constexpr uint32_t usart3RemapMsk = 3u << usart3RemapInd;
        static constexpr uint32_t usart3RemapNo = 0u << usart3RemapInd;
        static constexpr uint32_t usart3RemapPartical = 1u << usart3RemapInd;
        static constexpr uint32_t usart3RemapFull = 3u << usart3RemapInd;

        static constexpr uint32_t usart2RemapInd = 3;
        static constexpr uint32_t usart2RemapMsk = 1u << usart2RemapInd;

        static constexpr uint32_t usart1RemapInd = 2;
        static constexpr uint32_t usart1RemapMsk = 1u << usart1RemapInd;

        static constexpr uint32_t i2c1RemapInd = 1;
        static constexpr uint32_t i2c1RemapMsk = 1u << i2c1RemapInd;

        static constexpr uint32_t sp1RemapInd = 0;
        static constexpr uint32_t sp1RemapMsk = 1u << sp1RemapInd;

    };
private:
    static volatile Regs data[1];
};

} // namespace gpio
