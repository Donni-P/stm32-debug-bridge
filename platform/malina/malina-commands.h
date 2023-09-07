#include <global_resources.h>
#include <static_map.h>

namespace commands{
class CommandExecutor{
public:
    CommandExecutor(void){
        readPorts();
    }
    enum class CommandType{
        invalid,
        list,
        fan,
        power,
        host,
        elock,
        reset,
        help,
        save,
        R
    };
    struct flags{
        static constexpr uint16_t fan    = 0x0001;
        static constexpr uint16_t host   = 0x0002;
        static constexpr uint16_t edcl   = 0x0004;
        static constexpr uint16_t pwr    = 0x0008;
        static constexpr uint16_t rst    = 0x0010;
        static constexpr uint16_t clear  = 0x8000;
    };
    class hasher{
        public:
        constexpr size_t operator () (std::string_view x) const{
           size_t result = 0;
           for ( uint8_t c:x){
               result = ((result << 13) | (result >> (sizeof(std::size_t)*8-13))) ^ c;
           }
           return result;
        }
    };
    void push (char c){
        global::shellRx.pushSafe(c);
    }
    static void readPorts(void){
        using namespace config;
        powerOff();
        uint16_t pinCfg = global::flashconfig[0];
        if ( (pinCfg&flags::clear) != 0 ){
            portPins.hostMode.write(false);
            portPins.edclLock.write(false);
            portPins.fanPwm.write(true);
            portPins.fanEn.write(true);
        }else{
            if ( pinCfg & flags::fan ){
                portPins.fanPwm.write(true);
                portPins.fanEn.write(true);
            }else{
                portPins.fanPwm.write(false);
                portPins.fanEn.write(false);
            }
            if ( pinCfg & flags::host ){
                portPins.hostMode.write(true);
            }else{
                portPins.hostMode.write(false);
            }
            if ( pinCfg & flags::edcl ){
                portPins.edclLock.write(true);
            }else{
                portPins.edclLock.write(false);
            }
            if ( pinCfg & flags::pwr ){
                powerOn();
                if ( pinCfg & flags::rst ){
                    SysTick->CTRL = 0x00;
                    SysTick->LOAD = (SystemCoreClock/8/2)-1;
                    SysTick->VAL = 0x00;
                    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
                    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
                        ;
                    SysTick->CTRL = 0x00;
                    portPins.nRst.write(true);
                }
            }
        }
    }
    template<size_t L>
    void execute(size_t argc, const std::array<std::string_view, L> &argv){
        using std::string_view;
        using namespace std::literals;
        using namespace config;
        static constexpr string_view switchNo = "incorrect parameter. must be 0 or 1"sv;
        static constexpr string_view error = "unknown command : "sv;
        static constexpr string_view errorParam = "invalid param: "sv;
        static constexpr string_view setTo = " set to: "sv;
        static constexpr string_view list = "LIST FAN POWER HOST ELOCK RESET SAVE R"sv;
        static constexpr string_view help =
            "FAN <0|1> - turn fan on/off\r\n"
            "POWER <0|1> - turn power on/off\r\n"
            "HOST <0|1> - turn force host mode on/off\r\n"
            "ELOCK <0|1> - turn edcl on/off\r\n"
            "RESET <0|1> - turn nRST on/off\r\n"
            "SAVE - save control pins states\r\n"
            "R - full power cycle\r\n"
            ""sv;
        static constexpr staticMap::StaticMap<string_view, CommandType, 10, 4, hasher> commands(
            {
                {"LIST"sv,CommandType::list},
                {"FAN"sv,CommandType::fan},
                {"POWER"sv,CommandType::power},
                {"HOST"sv,CommandType::host},
                {"ELOCK"sv,CommandType::elock},
                {"RESET"sv,CommandType::reset},
                {"HELP"sv,CommandType::help},
                {"help"sv,CommandType::help},
                {"SAVE"sv,CommandType::save},
                {"R"sv,CommandType::R}
            }
        );
        CommandType c;
        {
            auto x = commands.find(argv[0]);
            if ( x == nullptr ){
                c = CommandType::invalid;
            }else{
                c = x->second;
            }
        }
        switch (c){
        case CommandType::invalid:
            for (uint8_t i:error)
                push(i);
            for (uint8_t i:argv[0])
                push(i);
            break;
        case CommandType::list:
            {
                for (uint8_t i:list)
                    push(i);
            }
            break;
        case CommandType::fan:
            {
                bool x;
                if (argc==2 && argv[1] == "0"){
                    x = false;
                }else if (argc==2 && argv[1] == "1"){
                    x = true;
                }else{
                    for (uint8_t i:errorParam)
                        push(i);
                    break;
                }
                for (uint8_t i:argv[0])
                    push(i);
                for (uint8_t i:setTo)
                    push(i);
                for (uint8_t i:argv[1])
                    push(i);
                portPins.fanPwm.write(x);
                portPins.fanEn.write(x);
            }
            break;
        case CommandType::power:
            {
                bool x;
                if (argc==2 && argv[1] == "0"){
                    x = false;
                }else if (argc==2 && argv[1] == "1"){
                    x = true;
                }else{
                    for (uint8_t i:switchNo)
                        push(i);
                    break;
                }
                for (uint8_t i:argv[0])
                    push(i);
                for (uint8_t i:setTo)
                    push(i);
                for (uint8_t i:argv[1])
                    push(i);
                if ( x )
                    powerOn();
                else
                    powerOff();
            }
            break;
        case CommandType::host:
            {
                bool x;
                if (argc==2 && argv[1] == "0"){
                    x = false;
                }else if (argc==2 && argv[1] == "1"){
                    x = true;
                }else{
                    for (uint8_t i:switchNo)
                        push(i);
                    break;
                }
                for (uint8_t i:argv[0])
                    push(i);
                for (uint8_t i:setTo)
                    push(i);
                for (uint8_t i:argv[1])
                    push(i);
                portPins.hostMode.write(x);
            }
            break;
        case CommandType::elock:
            {
                bool x;
                if (argc==2 && argv[1] == "0"){
                    x = false;
                }else if (argc==2 && argv[1] == "1"){
                    x = true;
                }else{
                    for (uint8_t i:switchNo)
                        push(i);
                    break;
                }
                for (uint8_t i:argv[0])
                    push(i);
                for (uint8_t i:setTo)
                    push(i);
                for (uint8_t i:argv[1])
                    push(i);
                portPins.edclLock.write(x);
            }
            break;
        case CommandType::reset:
            {
                bool x;
                if (argc==2 && argv[1] == "0"){
                    x = false;
                }else if (argc==2 && argv[1] == "1"){
                    x = true;
                }else{
                    for (uint8_t i:switchNo)
                        push(i);
                    break;
                }
                for (uint8_t i:argv[0])
                    push(i);
                for (uint8_t i:setTo)
                    push(i);
                for (uint8_t i:argv[1])
                    push(i);
                portPins.nRst.write(x);
            }
            break;
        case CommandType::help:
            {
                for (uint8_t i:help)
                    push(i);
            }
            break;
        case CommandType::save:
            {
                uint16_t pinCfg = 0;
                pinCfg |= portPins.fanPwm.read()?flags::fan:0;
                pinCfg |= portPins.hostMode.read()?flags::host:0;
                pinCfg |= portPins.edclLock.read()?flags::edcl:0;
                pinCfg |= portPins.pwrOn.read()?flags::pwr:0;
                pinCfg |= portPins.nRst.read()?flags::rst:0;
                if ( flash::Flash::unlock() ){
                    flash::Flash::pageErase(global::flashconfig);
                    flash::Flash::write(global::flashconfig,pinCfg);
                    flash::Flash::lock();
                }
            }
            break;
        case CommandType::R:
            {
                powerOff();
                SysTick->CTRL = 0x00;
                SysTick->LOAD = (SystemCoreClock/8/2)-1;
                SysTick->VAL = 0x00;
                SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
                while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
                    ;
                SysTick->CTRL = 0x00;
                powerOn();
                SysTick->CTRL = 0x00;
                SysTick->LOAD = (SystemCoreClock/8/2)-1;
                SysTick->VAL = 0x00;
                SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
                while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
                    ;
                SysTick->CTRL = 0x00;
                portPins.nRst.write(true);
            }
            break;
        }
        push('\r');
        push('\n');
    }
private:
    static void powerOn (){
        using namespace gpio;
        using namespace config;
        portPins.pwrOn.write(true);
        portPins.nRst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);

        portPins.pwrOn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.hostMode.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.edclLock.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.fanPwm.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.fanEn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);

        portPins.jtagTrst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.jtagHalt.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);

        global::jtagOut.configOutput<0>(OutputType::gen_pp,
                                        OutputSpeed::_10mhz);
        global::jtagOut.configOutput<1>(OutputType::gen_pp,
                                        OutputSpeed::_10mhz);
        global::jtagOut.configOutput<2>(OutputType::gen_pp,
                                        OutputSpeed::_10mhz);

        global::uartPins.configOutput<1>(OutputType::alt_pp,
                                         OutputSpeed::_10mhz); // TX
    }
    static void powerOff (){
        using namespace gpio;
        using namespace config;
        portPins.nRst.write(false);
        portPins.pwrOn.write(false);

        portPins.pwrOn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.nRst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);

        portPins.hostMode.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.edclLock.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.fanPwm.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.fanEn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);

        portPins.jtagTrst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.jtagHalt.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);

        global::jtagOut.configOutput<0>(OutputType::gen_pp,
                                        OutputSpeed::_10mhz);
        global::jtagOut.configOutput<1>(OutputType::gen_pp,
                                        OutputSpeed::_10mhz);
        global::jtagOut.configOutput<2>(OutputType::gen_pp,
                                        OutputSpeed::_10mhz);

        global::uartPins.configOutput<1>(OutputType::alt_pp,
                                         OutputSpeed::_10mhz); // TX
        /*
        portPins.pwrOn.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.nRst.configOutput(OutputType::gen_pp, OutputSpeed::_2mhz);
        portPins.hostMode.configInput(InputType::floating);
        portPins.edclLock.configInput(InputType::floating);
        portPins.jtagTrst.configInput(InputType::floating);
        portPins.jtagHalt.configInput(InputType::floating);
        portPins.fanPwm.configInput(InputType::floating);
        portPins.fanEn.configInput(InputType::floating);

        global::jtagOut.configInput<0>(InputType::floating);
        global::jtagOut.configInput<1>(InputType::floating);
        global::jtagOut.configInput<2>(InputType::floating);

        global::uartPins.configInput<1>(InputType::floating);
        */
    }
};
}
