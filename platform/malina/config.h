#pragma once

#include <gpio.h>
#include <flash.h>

namespace config{

struct PortPins{
    gpio::Pin<gpio::Port::b, 9> pwrOn;
    gpio::Pin<gpio::Port::b, 0> hostMode;
    gpio::Pin<gpio::Port::b, 1> edclLock;
    gpio::Pin<gpio::Port::a, 15> fanEn;
    gpio::Pin<gpio::Port::a, 6> fanPwm;
    gpio::Pin<gpio::Port::a, 8> nRst;
    gpio::Pin<gpio::Port::b, 10> jtagTrst;
    gpio::Pin<gpio::Port::b, 11> jtagHalt;
};

extern PortPins portPins;

static inline void ledOn(void){}

static inline void ledOff(void){}


static inline void PortsInit(void) {
    using namespace gpio;

    portPins.pwrOn.write(false);
    portPins.hostMode.write(false);
    portPins.edclLock.write(false);
    portPins.fanPwm.write(true);
    portPins.fanEn.write(true);
    portPins.nRst.write(false);
    Afio()->MAPR = Afio::MAPR::swjCfgSwdOnly;
    portPins.pwrOn.clockOn();
    portPins.pwrOn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.hostMode.clockOn();
    portPins.hostMode.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.edclLock.clockOn();
    portPins.edclLock.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.fanPwm.clockOn();
    portPins.fanPwm.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.fanEn.clockOn();
    portPins.fanEn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.nRst.clockOn();
    portPins.nRst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.jtagTrst.clockOn();
    portPins.jtagTrst.write(true);
    portPins.jtagTrst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
    portPins.jtagHalt.clockOn();
    portPins.jtagHalt.write(true);
    portPins.jtagHalt.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
}

static inline void Panic(void){}
static inline void configInit(void){
    using namespace usb::cdcPayload;
    LineCoding x = { 1000000, CharFormat::stopBit1, ParityType::none, DataBits::bits8 };
    setLineCoding( &x );
    applyLineCoding();
}

} // namespace global
