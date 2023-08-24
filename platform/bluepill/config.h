#pragma once

#include <gpio.h>
#include <static_map.h>

namespace config{


struct PortPins{
    gpio::Pin<gpio::Port::c, 13> led;
};

extern PortPins portPins;

static inline void ledOn(void){
    portPins.led.writeLow();
}

static inline void ledOff(void){
    portPins.led.writeHigh();
}

static inline void PortsInit(void) {
    portPins.led.clockOn();
    ledOff();
    portPins.led.configOutput(gpio::OutputType::gen_pp, gpio::OutputSpeed::_2mhz);
}

static inline void Panic(void){
    ledOn();
}

static inline void reset([[maybe_unused]]bool trts, [[maybe_unused]]bool srst){}

static inline void configInit(void){}

} // namespace global
