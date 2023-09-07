#include <string.h>
#include <usb_callbacks.h>
#include <usb_descriptors.h>

namespace usb {
namespace descriptor {
static const Device device(bcdVersion(2, 0, 0), Device::Class::misc,
                           Device::SubClass::iad, Device::Protocol::iad,
                           epSize::control, idVendor, idProduct,
                           bcdVersion(0, 0, 0),
                           static_cast<std::uint8_t>(StringIndex::manufacturer),
                           static_cast<std::uint8_t>(StringIndex::product),
                           static_cast<std::uint8_t>(StringIndex::serial), 1);

const DeviceConfiguration deviceConfig;

static const String<static_cast<char16_t>(LanguageCode::en_US)> lang;
static const auto manufacturer = VENDOR_STRING;
static const auto product = u"DJM (Debugger Jtag Module)"_sd;
static const auto uartInterfaceName = u"uart"_sd;
static const auto shellInterfaceName = u"shell"_sd;
static const auto jtagInterfaceName = u"jtag"_sd;
static const auto serial = [] {
    const char hexDigits[] = "0123456789ABCDEF";
    const uint8_t *uid = (uint8_t *)UID_BASE;
    auto x = u"XXXXXXXXXXXXXXXXXXXXXXXX"_sd;
    const unsigned uidLeft = 12;
    for (unsigned i = 0; i < uidLeft; ++i) {
        x.wString[i * 2] = hexDigits[uid[i] >> 4];
        x.wString[i * 2 + 1] = hexDigits[uid[i] & 0xf];
    }
    return x;
}();

static const Base *strings[static_cast<std::size_t>(StringIndex::last)] = {
    &lang,
    &manufacturer,
    &product,
    &serial,
    &uartInterfaceName,
    &shellInterfaceName,
    &jtagInterfaceName};

static const Qualifier qualifier(device.bcdUSB, device.bDeviceClass,
                                 device.bDeviceSubClass, device.bDeviceProtocol,
                                 device.bMaxPacketSize, 0);

const io::Endpoint endpoints[static_cast<std::size_t>(EndpointIndex::last)] = {
    {Endpoint::EpType::control, device.bMaxPacketSize, device.bMaxPacketSize,
     controlRxHandler, controlTxHandler, controlSetupHandler},
    {Endpoint::EpType::interrupt, 0,
     static_cast<std::uint8_t>(deviceConfig.uart.commEp.wMaxPacketSize),
     nullptr, nullptr, nullptr},
    {Endpoint::EpType::bulk,
     static_cast<std::uint8_t>(deviceConfig.uart.dataRxEp.wMaxPacketSize),
     static_cast<std::uint8_t>(deviceConfig.uart.dataTxEp.wMaxPacketSize),
     uartRxHandler, uartTxHandler, nullptr},
    {Endpoint::EpType::interrupt, 0,
     static_cast<std::uint8_t>(deviceConfig.shell.commEp.wMaxPacketSize),
     nullptr, nullptr, nullptr},
    {Endpoint::EpType::bulk,
     static_cast<std::uint8_t>(deviceConfig.shell.dataRxEp.wMaxPacketSize),
     static_cast<std::uint8_t>(deviceConfig.shell.dataTxEp.wMaxPacketSize),
     shellRxHandler, shellTxHandler, nullptr},
    {Endpoint::EpType::interrupt, 0,
     static_cast<std::uint8_t>(deviceConfig.jtag.commEp.wMaxPacketSize),
     nullptr, nullptr, nullptr},
    {Endpoint::EpType::bulk,
     static_cast<std::uint8_t>(deviceConfig.jtag.dataRxEp.wMaxPacketSize),
     static_cast<std::uint8_t>(deviceConfig.jtag.dataTxEp.wMaxPacketSize),
     jtagRxHandler, jtagTxHandler, nullptr}};

constexpr uint32_t buff_size = []{uint32_t result=0; for (auto i:endpoints){result += i.rxSize+i.txSize;}return result;}();

static_assert (buff_size<512-sizeof(io::bTableEntity) * 8);

uint16_t get(uint16_t wValue, const uint8_t *&payload) {
    uint16_t size = 0;
    usb::descriptor::Type type =
        static_cast<usb::descriptor::Type>(wValue >> 8);
    uint8_t index = (wValue & 0xff);
    switch (type) {
    case usb::descriptor::Type::device:
        payload =
            static_cast<const uint8_t *>(static_cast<const void *>(&device));
        size = sizeof(device);
        break;

    case usb::descriptor::Type::configuration:
        payload = static_cast<const uint8_t *>(
            static_cast<const void *>(&deviceConfig));
        size = sizeof(deviceConfig);
        break;
    case usb::descriptor::Type::qualifier:
        payload =
            static_cast<const uint8_t *>(static_cast<const void *>(&qualifier));
        size = sizeof(qualifier);
        break;

    case usb::descriptor::Type::string:
        if (index < static_cast<uint8_t>(StringIndex::last)) {
            payload = static_cast<const uint8_t *>(
                static_cast<const void *>(strings[index]));
            size = strings[index]->bLength;

        } else {
            return 0;
        }
        break;

    default:
        return 0;
    }
    return size;
}

} // namespace descriptor
} // namespace usb
