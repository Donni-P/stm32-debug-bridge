
#include <usb.h>

#include <fifo.h>
#include <global_resources.h>
#include <gpio.h>

#include <algorithm>

extern "C" void __terminate();

namespace usb {

constexpr uint16_t istrMAsk = USB_ISTR_CTR | USB_ISTR_RESET | USB_ISTR_SUSP |
                              USB_ISTR_WKUP | USB_ISTR_DIR | USB_ISTR_EP_ID_Msk;

struct DeviceState {
    enum class State { reset = 0x00, addressSet = 0x01, configured = 0x02 };
    State state;
    uint8_t address;
    uint8_t configuration;
};
static DeviceState deviceState;
bool uartTxProcessing;
bool shellTxProcessing;
bool jtagTxProcessing;

struct ControlState {
    enum class State { idle, rx, tx, txZlp, txLast, statusIn, statusOut };
    State state;
    Setup setup;
    uint8_t *currentPayload;
    const uint8_t *payloadTx;
    uint16_t payloadLeft;
};

static ControlState controlState;

namespace epState {

enum class txState : uint16_t {
    dis = USB_EP_TX_DIS,
    stall = USB_EP_TX_STALL,
    nak = USB_EP_TX_NAK,
    valid = USB_EP_TX_VALID
};
enum class rxState : uint16_t {
    dis = USB_EP_RX_DIS,
    stall = USB_EP_RX_STALL,
    nak = USB_EP_RX_NAK,
    valid = USB_EP_RX_VALID
};

static inline void setRx(descriptor::EndpointIndex epNum, rxState state) {
    auto &epReg = io::epRegs(static_cast<ptrdiff_t>(epNum));
    auto epReg_ = epReg;
    epReg = ((epReg_ ^ static_cast<uint16_t>(state)) &
             (USB_EPREG_MASK | USB_EPRX_STAT_Msk)) |
            (USB_EP_CTR_RX | USB_EP_CTR_TX);
}

static inline rxState getRx(descriptor::EndpointIndex epNum) {
    auto &epReg = io::epRegs(static_cast<ptrdiff_t>(epNum));
    auto state = epReg & USB_EPRX_STAT_Msk;
    return static_cast<rxState>(state);
}

static inline void setTx(descriptor::EndpointIndex epNum, txState state) {
    auto &epReg = io::epRegs(static_cast<ptrdiff_t>(epNum));
    auto epReg_ = epReg;
    epReg = ((epReg_ ^ static_cast<uint16_t>(state)) &
             (USB_EPREG_MASK | USB_EPTX_STAT_Msk)) |
            (USB_EP_CTR_RX | USB_EP_CTR_TX);
}

static inline txState getTx(descriptor::EndpointIndex epNum) {
    auto &epReg = io::epRegs(static_cast<ptrdiff_t>(epNum));
    auto state = epReg & USB_EPTX_STAT_Msk;
    return static_cast<txState>(state);
}

} // namespace epState

static inline void endpointSetStall(descriptor::EndpointIndex epNum,
                                    descriptor::Endpoint::Direction dir,
                                    bool stall) {
    auto &epReg = io::epRegs(static_cast<ptrdiff_t>(epNum));
    auto epReg_ = epReg;
    if ((epReg_ & USB_EP_T_FIELD) != USB_EP_ISOCHRONOUS) {
        if (dir == descriptor::Endpoint::Direction::in) {
            if (epState::getTx(epNum) != epState::txState::dis) {
                if (stall) {
                    epState::setTx(epNum, epState::txState::stall);
                } else {
                    epState::setTx(epNum, epState::txState::nak);
                }
            }
        } else {
            if (epState::getRx(epNum) != epState::rxState::dis) {
                if (stall) {
                    epState::setRx(epNum, epState::rxState::stall);
                } else {
                    epState::setRx(epNum, epState::rxState::valid);
                }
            }
        }
    }
}
static inline bool endpointGetStall(descriptor::EndpointIndex epNum,
                                    descriptor::Endpoint::Direction dir) {
    if (dir == descriptor::Endpoint::Direction::in) {
        return (epState::getTx(epNum) == epState::txState::stall);
    }
    return (epState::getRx(epNum) == epState::rxState::stall);
}

static volatile io::bTableEntity *const bTable =
    reinterpret_cast<io::bTableEntity *>(static_cast<uintptr_t>(USB_PMAADDR));

static inline const io::pBufferData *rxBuf(std::uint8_t epNum) {
    return reinterpret_cast<io::pBufferData *>(
        static_cast<uintptr_t>(USB_PMAADDR + bTable[epNum].rxOffset * 2));
}
static inline io::pBufferData *txBuf(std::uint8_t epNum) {
    return reinterpret_cast<io::pBufferData *>(
        static_cast<uintptr_t>(USB_PMAADDR + bTable[epNum].txOffset * 2));
}

static int read(descriptor::EndpointIndex epNum, void *buf, size_t bufSize) {
    const ptrdiff_t epNum_ = static_cast<ptrdiff_t>(epNum);
    const io::pBufferData *epBuf = rxBuf(epNum_);
    const uint16_t rxCount = bTable[epNum_].rxCount;
    const uint16_t epBytesCount = rxCount & USB_COUNT0_RX_COUNT0_RX;
    uint16_t *bufP = reinterpret_cast<uint16_t *>(buf);
    if (bufSize < epBytesCount) {
        return -1;
    }
    bTable[epNum_].rxCount =
        rxCount & static_cast<uint16_t>(~USB_COUNT0_RX_COUNT0_RX);
    for (unsigned i = epBytesCount / 2; i > 0; --i) {
        *bufP++ = (epBuf++)->data;
    }
    if (epBytesCount % 2) {
        *reinterpret_cast<uint8_t *>(bufP) = static_cast<uint8_t>(epBuf->data);
    }
    epState::setRx(epNum, epState::rxState::valid);
    return epBytesCount;
}

static size_t write(descriptor::EndpointIndex epNum, const void *buf,
                    size_t count) {
    const ptrdiff_t epNum_ = static_cast<ptrdiff_t>(epNum);
    io::pBufferData *epBuf = txBuf(epNum_);
    const uint16_t *bufP = reinterpret_cast<const uint16_t *>(buf);
    const size_t txSize = descriptor::endpoints[epNum_].txSize;
    if (count > txSize) {
        count = txSize;
    }
    for (unsigned i = (count + 1) / 2; i > 0; --i) {
        (epBuf++)->data = *bufP++;
    }
    bTable[epNum_].txCount = count;
    epState::setTx(epNum, epState::txState::valid);
    return count;
}

static size_t readToFifo(descriptor::EndpointIndex epNum,
                         fifo::Fifo<uint8_t, global::cdcFifoLenTx> &data) {
    const ptrdiff_t epNum_ = static_cast<ptrdiff_t>(epNum);
    const io::pBufferData *epBuf = rxBuf(epNum_);
    const uint16_t rxCount = bTable[epNum_].rxCount;
    const uint16_t epBytesCount = rxCount & USB_COUNT0_RX_COUNT0_RX;

    if (epBytesCount > data.capacity() - data.size()) {
        return 0;
    }

    bTable[epNum_].rxCount =
        rxCount & static_cast<uint16_t>(~USB_COUNT0_RX_COUNT0_RX);
    for (size_t wordsLeft = epBytesCount / 2; wordsLeft > 0; --wordsLeft) {
        uint16_t x = *epBuf;
        data.write(static_cast<const uint8_t *>(static_cast<const void *>(&x)), 2);
        epBuf++;
    }
    if (epBytesCount % 2) {
        data.push(epBuf->data);
    }
    if(data.capacity() - data.size() >= descriptor::endpoints[epNum_].rxSize){
        epState::setRx(epNum, epState::rxState::valid);
    }
    return epBytesCount;
}

size_t writeFromFifo(descriptor::EndpointIndex epNum,
                     fifo::Fifo<uint8_t, global::cdcFifoLenRx> &data) {
    const ptrdiff_t epNum_ = static_cast<ptrdiff_t>(epNum);
    size_t count =
        std::min<size_t>(descriptor::endpoints[epNum_].txSize, data.size());
    io::pBufferData *epBuf = txBuf(epNum_);
    for (size_t wordsLeft = count / 2; wordsLeft > 0; --wordsLeft) {
	uint16_t x = data.pop();
	x |= static_cast<uint16_t>(data.pop())<<8;
        *epBuf++ = x;
    }
    if (count % 2) {
        epBuf->data = data.pop();
    }
    bTable[epNum_].txCount = count;
    epState::setTx(epNum, epState::txState::valid);
    return count;
}

namespace control {

enum class status { ack = 0x00, nak = 0x01, fail = 0x02 };

static status processRequest() {
    if ((controlState.setup.type == Setup::type_class) &&
        (controlState.setup.recipient == Setup::recipient_interface)) {
        if ((controlState.setup.wIndex ==
             static_cast<uint16_t>(descriptor::InterfaceIndex::jtag)) ||
            (controlState.setup.wIndex ==
             static_cast<uint16_t>(descriptor::InterfaceIndex::shell))) {
            return status::ack;
        } else if (controlState.setup.wIndex ==
                   static_cast<uint16_t>(descriptor::InterfaceIndex::uart)) {
            switch (static_cast<cdc::Request>(controlState.setup.bRequest)) {
            case cdc::Request::set_control_line_state:
                cdcPayload::setControlLineState(controlState.setup.wValue);
                return status::ack;
            case cdc::Request::set_line_coding: {
                const cdcPayload::LineCoding *lineCoding =
                    static_cast<cdcPayload::LineCoding *>(
                        static_cast<void *>(controlState.setup.payload));
                if (controlState.setup.wLength ==
                    sizeof(cdcPayload::LineCoding)) {
                    if (cdcPayload::setLineCoding(lineCoding)) {
                        return status::ack;
                    }
                }
                return status::fail;
            }
            case cdc::Request::get_line_coding:
                if (controlState.setup.wLength ==
                    sizeof(cdcPayload::LineCoding)) {
                    controlState.payloadTx = static_cast<uint8_t *>(
                        static_cast<void *>(&global::uartLineCoding));
                    return status::ack;
                }
                return status::fail;
            default:
                return status::fail;
            }
        }
    } else if (controlState.setup.type == Setup::type_standard) {
        switch (controlState.setup.recipient) {
        case Setup::recipient_device:
            switch (static_cast<Setup::Request>(controlState.setup.bRequest)) {
            case Setup::Request::get_configuration:
                controlState.setup.payload[0] = deviceState.configuration;
                controlState.payloadTx = controlState.setup.payload;
                controlState.payloadLeft = 1;
                return status::ack;
            case Setup::Request::get_descriptor:
                controlState.payloadLeft = descriptor::get(
                    controlState.setup.wValue, controlState.payloadTx);
                if (controlState.payloadLeft != 0) {
                    return status::ack;
                }
                return status::fail;
            case Setup::Request::get_status:
                controlState.setup.payload[0] = deviceState.configuration;
                controlState.setup.payload[1] = deviceState.configuration;
                controlState.payloadTx = controlState.setup.payload;
                controlState.payloadLeft = 2;
                return status::ack;
            // TODO: в оригинале были какие-то лютые сложности, попробовал
            // выставлять сразу
            case Setup::Request::set_address:
                deviceState.address =
                    controlState.setup.wValue & USB_DADDR_ADD_Msk;
                return status::ack;
            case Setup::Request::set_configuration: {
                uint8_t device_configuration = controlState.setup.wValue & 0xff;
                if (device_configuration == 1) {
                    deviceState.configuration = device_configuration;
                    return status::ack;
                }
                break;
            }
            default:
                break;
            }
            return status::fail;
        case Setup::recipient_interface:
            if (controlState.setup.bRequest ==
                static_cast<uint8_t>(Setup::Request::get_status)) {
                controlState.setup.payload[0] = 0;
                controlState.setup.payload[1] = 0;
                controlState.payloadTx = controlState.setup.payload;
                controlState.payloadLeft = 2;
                return status::ack;
            }
            break;
        case Setup::recipient_endpoint: {
            descriptor::EndpointIndex epNum =
                static_cast<descriptor::EndpointIndex>(
                    controlState.setup.wIndex &
                    ~(descriptor::Endpoint::directionIn));
            descriptor::Endpoint::Direction dir =
                static_cast<descriptor::Endpoint::Direction>(
                    controlState.setup.wIndex &
                    descriptor::Endpoint::directionIn);
            switch (static_cast<Setup::Request>(controlState.setup.bRequest)) {
            case Setup::Request::set_feature:
            case Setup::Request::clear_feature: {
                bool epStall =
                    (static_cast<Setup::Request>(controlState.setup.bRequest) ==
                     Setup::Request::set_feature);
                endpointSetStall(epNum, dir, epStall);
                return status::ack;
            }
            case Setup::Request::get_status:
                controlState.setup.payload[0] = endpointGetStall(epNum, dir);
                controlState.setup.payload[1] = 0;
                controlState.payloadTx = controlState.setup.payload;
                controlState.payloadLeft = 2;
                return status::ack;
            default:
                break;
            }
            break;
        }
        }
    }
    return status::fail;
}

static inline void epStall() {
    epState::setTx(descriptor::EndpointIndex::control, epState::txState::stall);
    epState::setRx(descriptor::EndpointIndex::control, epState::rxState::stall);
    controlState.state = ControlState::State::idle;
}
void setAddr() {
    if (deviceState.address != 0) {
        USB->DADDR = USB_DADDR_EF | deviceState.address;
    }
}
void txDispatch() {
    size_t bytesSent = 0;
    switch (controlState.state) {
    case ControlState::State::tx:
    case ControlState::State::txZlp:
        bytesSent = write(descriptor::EndpointIndex::control,
                          controlState.payloadTx, controlState.payloadLeft);
        controlState.payloadTx += bytesSent;
        controlState.payloadLeft -= bytesSent;
        if (controlState.payloadLeft == 0) {
            if ((controlState.state == ControlState::State::tx) ||
                (bytesSent !=
                 descriptor::endpoints[static_cast<uint8_t>(
                                           descriptor::EndpointIndex::control)]
                     .txSize)) {
                controlState.state = ControlState::State::txLast;
            }
        }
        break;
    case ControlState::State::txLast:
        controlState.state = ControlState::State::statusOut;
        break;
    case ControlState::State::statusIn:
        controlState.state = ControlState::State::idle;
        setAddr();
        break;

    default:
        __terminate();
        break;
    }
}
void rxDispatch() {
    size_t setupSize;
    size_t payloadBytesReceived = 0;
    switch (controlState.state) {
    case ControlState::State::idle:
        setupSize =
            read(descriptor::EndpointIndex::control,
                 static_cast<void *>(&controlState.setup), sizeof(Setup));
        if (setupSize != (sizeof(Setup) - sizeof(Setup::payload))) {
            epStall();
            return;
        } else {
            controlState.currentPayload = controlState.setup.payload;
            controlState.payloadLeft = controlState.setup.wLength;
            if ((controlState.setup.direction ==
                 Setup::direction_host_to_device) &&
                (controlState.setup.wLength != 0)) {
                if (controlState.setup.wLength > sizeof(Setup::payload)) {
                    epStall();
                } else {
                    controlState.state = ControlState::State::rx;
                }
                return;
            }
        }
        break;
    case ControlState::State::rx:
        payloadBytesReceived =
            read(descriptor::EndpointIndex::control,
                 controlState.currentPayload, controlState.setup.wLength);
        if (controlState.setup.wLength != payloadBytesReceived) {
            controlState.currentPayload += payloadBytesReceived;
            controlState.payloadLeft -= payloadBytesReceived;
            return;
        }
        break;
    case ControlState::State::statusOut:
        read(descriptor::EndpointIndex::control, nullptr, 0);
        controlState.state = ControlState::State::idle;
        break;
    default:
        epStall();
        return;
    }
    controlState.currentPayload = controlState.setup.payload;
    controlState.payloadLeft = sizeof(Setup::payload);
    switch (processRequest()) {
    case status::ack:
        if (controlState.setup.direction == Setup::direction_device_to_host) {
            if (controlState.payloadLeft < controlState.setup.wLength) {
                controlState.state = ControlState::State::txZlp;
            } else {
                if (controlState.payloadLeft > controlState.setup.wLength) {
                    controlState.payloadLeft = controlState.setup.wLength;
                }
                controlState.state = ControlState::State::tx;
            }
            txDispatch();
        } else {
            write(descriptor::EndpointIndex::control, nullptr, 0);
            controlState.state = ControlState::State::statusIn;
        }
        break;
    case status::nak:
        controlState.state = ControlState::State::statusIn;
        break;
    default:
        epStall();
    }
}

} // namespace control
void controlRxHandler() { control::rxDispatch(); }
void controlTxHandler() { control::txDispatch(); }
void controlSetupHandler() {
    controlState.state = ControlState::State::idle;
    control::rxDispatch();
}

void uartRxHandler() {
    if (!cdcPayload::isPendingApply()) {
        readToFifo(descriptor::EndpointIndex::uartData, global::uartTx);
    }
}

void uartTxHandler() {
    if (!global::uartRx.empty()) {
        writeFromFifo(descriptor::EndpointIndex::uartData, global::uartRx);
    }else{
        uartTxProcessing = false;
    }
}
void uartInterruptHandler() { ; }

void shellRxHandler() {
    readToFifo(descriptor::EndpointIndex::shellData, global::shellTx);
}
void shellTxHandler() {
    if (!global::shellRx.empty()) {
        writeFromFifo(descriptor::EndpointIndex::shellData, global::shellRx);
    }else{
        shellTxProcessing = false;
    }
}
void shellInterruptHandler() { ; }

void jtagRxHandler() {
    readToFifo(descriptor::EndpointIndex::jtagData, global::jtagTx);
}
void jtagTxHandler() {
    if (!global::jtagRx.empty()) {
        writeFromFifo(descriptor::EndpointIndex::jtagData, global::jtagRx);
    }else{
        jtagTxProcessing = false;
    }
}
void jtagInterruptHandler() { ; }

void regenerateTx(void) {
    if (auto epNum = descriptor::EndpointIndex::uartData;
        epState::getRx(epNum) == epState::rxState::nak &&
        ((global::uartTx.capacity() - global::uartTx.size()) >=
         descriptor::endpoints[static_cast<uint8_t>(epNum)].rxSize)) {
        epState::setRx(epNum, epState::rxState::valid);
    }
    if (auto epNum = descriptor::EndpointIndex::shellData;
        epState::getRx(epNum) == epState::rxState::nak &&
        ((global::shellTx.capacity() - global::shellTx.size()) >=
         descriptor::endpoints[static_cast<uint8_t>(epNum)].rxSize)) {
        epState::setRx(epNum, epState::rxState::valid);
    }
    if (auto epNum = descriptor::EndpointIndex::jtagData;
        epState::getRx(epNum) == epState::rxState::nak &&
        ((global::jtagTx.capacity() - global::jtagTx.size()) >=
         descriptor::endpoints[static_cast<uint8_t>(epNum)].rxSize)) {
        epState::setRx(epNum, epState::rxState::valid);
    }
}

constexpr uint16_t convertEpType(descriptor::Endpoint::EpType t) {
    uint16_t epType = 0;
    switch (t) {
    case descriptor::Endpoint::EpType::control:
        epType = USB_EP_CONTROL;
        break;
    case descriptor::Endpoint::EpType::isochronous:
        epType = USB_EP_ISOCHRONOUS;
        break;
    case descriptor::Endpoint::EpType::bulk:
        epType = USB_EP_BULK;
        break;
    case descriptor::Endpoint::EpType::interrupt:
        epType = USB_EP_INTERRUPT;
        break;
    }
    return epType;
}

void init(void) {
    /* Force USB re-enumeration */

    global::usbPins.write(false, false);
    global::usbPins.configOutput<1>(gpio::OutputType::gen_pp,
                                    gpio::OutputSpeed::_2mhz);

    SysTick->CTRL = 0x00;
    SysTick->LOAD = (SystemCoreClock/8/100)-1;
    SysTick->VAL = 0x00;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;


    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
        ;
    SysTick->CTRL = 0x00;

    global::usbPins.configInput<0>(gpio::InputType::floating);
    global::usbPins.configInput<1>(gpio::InputType::floating);

    USB->CNTR = USB_CNTR_FRES;

    USB->ISTR = 0;
    USB->BTABLE = 0;
    USB->DADDR = 0;

    NVIC_EnableIRQ(USB_LP_IRQn);
    NVIC_DisableIRQ(USB_HP_IRQn);

    USB->CNTR = USB_CNTR_RESETM;
}

void reset(void) {
    deviceState.state = DeviceState::State::reset;
    deviceState.address = 0;
    deviceState.configuration = 0;
    uint16_t offset = sizeof(io::bTableEntity) * 8;
    using descriptor::endpoints;
    for (std::ptrdiff_t epNum = 0;
         epNum < static_cast<std::ptrdiff_t>(descriptor::EndpointIndex::last);
         epNum++) {

        bTable[epNum].txOffset = offset;
        bTable[epNum].txCount = 0;
        offset += endpoints[epNum].txSize;

        bTable[epNum].rxOffset = offset;
        if (endpoints[epNum].rxSize > io::smallBlockSizeLimit) {
            bTable[epNum].rxCount =
                ((endpoints[epNum].rxSize / io::largeBlockSize - 1)
                 << USB_COUNT0_RX_NUM_BLOCK_Pos) |
                USB_COUNT0_RX_BLSIZE;
        } else {
            bTable[epNum].rxCount =
                (endpoints[epNum].rxSize / io::smallBlockSize)
                << USB_COUNT0_RX_NUM_BLOCK_Pos;
        }
        offset += endpoints[epNum].rxSize;
        uint16_t tmp = io::epRegs(epNum);
        tmp &= USB_EP_T_FIELD;
        tmp ^= USB_EP_RX_VALID | USB_EP_TX_NAK;
        tmp |= epNum | convertEpType(endpoints[epNum].type);
        io::epRegs(epNum) = tmp;
    }
    USB->CNTR =
        USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
    USB->DADDR = USB_DADDR_EF;
}

static inline bool isAcmReadyToSend(descriptor::InterfaceIndex ind) {
    switch (ind) {
    case descriptor::InterfaceIndex::uart:
        return !uartTxProcessing && epState::getTx(descriptor::EndpointIndex::uartData) ==
               epState::txState::nak;
        break;
    case descriptor::InterfaceIndex::shell:
        return !shellTxProcessing && epState::getTx(descriptor::EndpointIndex::shellData) ==
               epState::txState::nak;
        break;
    case descriptor::InterfaceIndex::jtag:
        return !jtagTxProcessing && epState::getTx(descriptor::EndpointIndex::jtagData) ==
               epState::txState::nak;
        break;
    default:
        return false;
        break;
    }
}

bool sendFromFifo(descriptor::InterfaceIndex ind,
                  fifo::Fifo<std::uint8_t, global::cdcFifoLenRx> &buf) {
    if (isAcmReadyToSend(ind)) {
        descriptor::EndpointIndex ep = [](descriptor::InterfaceIndex ind) {
            switch (ind) {
            case descriptor::InterfaceIndex::uart:
                uartTxProcessing = true;
                return descriptor::EndpointIndex::uartData;

            case descriptor::InterfaceIndex::shell:
                shellTxProcessing = true;
                return descriptor::EndpointIndex::shellData;

            case descriptor::InterfaceIndex::jtag:
                jtagTxProcessing = true;
                return descriptor::EndpointIndex::jtagData;

            default:
                return descriptor::EndpointIndex::last;
            }
        }(ind);
        writeFromFifo(ep, buf);
        return true;
    }
    return false;
}

void polling(void) {
    using usb::descriptor::endpoints;
    uint16_t istr = USB->ISTR & istrMAsk;
    if (istr != 0) {
        if (istr & USB_ISTR_CTR) {
            uint16_t epNum = (istr & USB_ISTR_EP_ID_Msk) >> USB_ISTR_EP_ID_Pos;
            auto epReg = usb::io::epRegs(epNum);
            usb::io::epRegs(epNum) =
                epReg &
                (USB_EP_T_FIELD_Msk | USB_EP_KIND_Msk | USB_EPADDR_FIELD_Msk);
            epReg &= (USB_EP_CTR_TX_Msk | USB_EP_CTR_RX_Msk | USB_EP_SETUP_Msk);
            if (epReg & USB_EP_CTR_TX) {
                if (endpoints[epNum].txHandler) {
                    endpoints[epNum].txHandler();
                }
            } else if (epReg & USB_EP_SETUP) {
                if (endpoints[epNum].setupHandler) {
                    endpoints[epNum].setupHandler();
                }
            } else {
                if (endpoints[epNum].rxHandler) {
                    endpoints[epNum].rxHandler();
                }
            }
        } else if (istr & USB_ISTR_RESET) {
            USB->ISTR = (uint16_t)(~USB_ISTR_RESET);
            usb::reset();
        } else if (istr & USB_ISTR_SUSP) {
            USB->ISTR = (uint16_t)(~USB_ISTR_SUSP);
            USB->CNTR = USB->CNTR | USB_CNTR_FSUSP;
            USB->DADDR = USB_DADDR_EF;
        } else if (istr & USB_ISTR_WKUP) {
            USB->ISTR = (uint16_t)(~USB_ISTR_WKUP);
            USB->CNTR = USB->CNTR & ~USB_CNTR_FSUSP;
        }
    }
}

} // namespace usb
extern "C" void USB_LP_IRQHandler() {
    usb::polling();
}
