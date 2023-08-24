#pragma once

#include <usb_cdc.h>
#include <usb_std.h>

namespace usb {
namespace descriptor {
enum class StringIndex {
    none = 0x00,
    lang = 0x00,
    manufacturer,
    product,
    serial,
    uartInterfaceName,
    shellInterfaceName,
    jtagInterfaceName,
    last
};
enum class EndpointIndex {
    control = 0x00,
    uartInterrupt,
    uartData,
    shellInterrupt,
    shellData,
    jtagInterrupt,
    jtagData,
    last
};
namespace epSize {
constexpr std::uint16_t control = 16;
constexpr std::uint16_t interrupt = 16;
constexpr std::uint16_t small = 16;
constexpr std::uint16_t large = 128;
} // namespace epSize

constexpr std::uint16_t idVendor = VENDOR_ID;
constexpr std::uint16_t idProduct = PRODUCT_ID;

enum class InterfaceIndex { uart = 0, shell = 2, jtag = 4 };

struct DeviceConfiguration : Configuration {

    struct SingleAcm {
        Iad iad;
        Interface comm;
        cdc::descriptor::Header header;
        cdc::descriptor::CallMgmt mgmt;
        cdc::descriptor::Acm acm;
        cdc::descriptor::Union union_;
        Endpoint commEp;
        Interface data;
        Endpoint dataRxEp;
        Endpoint dataTxEp;
        constexpr SingleAcm(InterfaceIndex iInd, EndpointIndex ecInd,
                            EndpointIndex edInd, bool realUart)
            : iad(static_cast<uint8_t>(iInd), 2,
                  static_cast<uint8_t>(Device::Class::comm), cdc::Subclass::acm,
                  cdc::Protocol::protocolDefault,
                  static_cast<uint8_t>(StringIndex::none)),
              comm(static_cast<uint8_t>(iInd), 0, 1, Device::Class::comm,
                   cdc::Subclass::acm, cdc::Protocol::protocolDefault,
                   static_cast<uint8_t>(StringIndex::uartInterfaceName)),
              header(), mgmt(0, static_cast<uint8_t>(iInd) + 1),
              acm(realUart ? cdc::acmCapability::default_ : 0),
              union_(static_cast<uint8_t>(iInd),
                     static_cast<uint8_t>(iInd) + 1),
              commEp(Endpoint::directionIn | static_cast<uint8_t>(ecInd),
                     Endpoint::EpType::interrupt, epSize::interrupt,
                     cdc::linesPollingInterval),
              data(static_cast<uint8_t>(iInd) + 1, 0, 2,
                   Device::Class::cdc_data, cdc::Subclass::none,
                   cdc::Protocol::none,
                   static_cast<uint8_t>(StringIndex::none)),
              dataRxEp(Endpoint::directionOut | static_cast<uint8_t>(edInd),
                       Endpoint::EpType::bulk, epSize::small, 0),
              dataTxEp(Endpoint::directionIn | static_cast<uint8_t>(edInd),
                       Endpoint::EpType::bulk, realUart?epSize::large:epSize::small, 0) {}
    } __attribute__((packed));

    SingleAcm uart;
    SingleAcm shell;
    SingleAcm jtag;

    constexpr DeviceConfiguration()
        : Configuration(sizeof(*this), cdc::numPorts * 2, 1,
                        static_cast<uint8_t>(StringIndex::none),
                        Configuration::bmAttrReserved, 100_ma),
          uart(InterfaceIndex::uart, EndpointIndex::uartInterrupt,
               EndpointIndex::uartData, true),
          shell(InterfaceIndex::shell, EndpointIndex::shellInterrupt,
                EndpointIndex::shellData, false),
          jtag(InterfaceIndex::jtag, EndpointIndex::jtagInterrupt,
               EndpointIndex::jtagData, false) {}

} __attribute__((packed));
extern const io::Endpoint
    endpoints[static_cast<std::size_t>(EndpointIndex::last)];

uint16_t get(uint16_t wValue, const uint8_t *&payload);
} // namespace descriptor
} // namespace usb
