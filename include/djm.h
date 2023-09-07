#pragma once

#include <cstdint>
#include <array>
#include <utility>

namespace djm{
template < class CE, std::size_t bufLen=128 >
    requires requires (CE ce, uint16_t count, uint8_t * buffer){
        ce.execute_scanIo(count,buffer);
        ce.execute_scanI(count,buffer);
        ce.execute_scanO(count,buffer);
        ce.execute_move(count,buffer);
        ce.execute_waitTicks(count);
        ce.execute_waitTime(count);
        ce.execute_setSpeed(count);
        ce.execute_B();
        ce.execute_b();
        ce.execute_R();
        ce.execute_0();
        ce.execute_1();
        ce.execute_2();
        ce.execute_3();
        ce.execute_4();
        ce.execute_5();
        ce.execute_6();
        ce.execute_7();
        ce.execute_r();
        ce.execute_t();
        ce.execute_s();
        ce.execute_u();
        ce.write(buffer,count);
    }
class Packets : public CE{
public:
    enum class State : uint8_t{
        normal,
        len1,
        len2,
        data
    };
    enum class Command : uint8_t{
        scanIo,
        scanI,
        scanO,
        speed,
        move,
        waitTicks,
        sleep,
    };
    void exec ( uint8_t c ){
        switch(state){
            case State::normal:
                switch (c) {
                    case 'B':
                        CE::execute_B();
                        break;
                    case 'b':
                        CE::execute_b();
                        break;
                    case 'R':
                        CE::execute_R();
                        break;
                    case '0':
                        CE::execute_0();
                        break;
                    case '1':
                        CE::execute_1();
                        break;
                    case '2':
                        CE::execute_2();
                        break;
                    case '3':
                        CE::execute_3();
                        break;
                    case '4':
                        CE::execute_4();
                        break;
                    case '5':
                        CE::execute_5();
                        break;
                    case '6':
                        CE::execute_6();
                        break;
                    case '7':
                        CE::execute_7();
                        break;
                    case 'r':
                        CE::execute_r();
                        break;
                    case 't':
                        CE::execute_t();
                        break;
                    case 's':
                        CE::execute_s();
                        break;
                    case 'u':
                        CE::execute_u();
                        break;
                    case 'h':
                        CE::write(i_am_alive_string,
                                             sizeof(i_am_alive_string) - 1);
                        break;
                    case '@':
                        state = State::len1;
                        cmd = Command::scanIo;
                        break;
                    case '$':
                        state = State::len1;
                        cmd = Command::scanI;
                        break;
                    case '#':
                        state = State::len1;
                        cmd = Command::scanO;
                        break;
                    case '%':
                        state = State::len1;
                        cmd = Command::speed;
                        break;
                    case '!':
                        state = State::len1;
                        cmd = Command::move;
                        break;
                    case '_':
                        state = State::len1;
                        cmd = Command::waitTicks;
                        break;
                    case '*':
                        state = State::len1;
                        cmd = Command::sleep;
                        break;

                    default:
                        break;
                }
                break;
            case State::len1:
                count_ = c;
                state = State::len2;
                break;
            case State::len2:
                count_ |= ((uint16_t)c)<<8;
                ind_ = 0;
                left_ = count_/8 + (count_%8?1:0);
                switch( cmd ){
                    case Command::scanIo:
                    case Command::scanO:
                    case Command::move:
                        state = State::data;
                        break;
                    case Command::scanI:
                        CE::execute_scanI(count_,buff_.data());
                        CE::write(buff_.data(),left_);
                        state = State::normal;
                        break;
                    case Command::speed:
                        CE::execute_setSpeed(count_);
                        state = State::normal;
                        break;
                    case Command::waitTicks:
                        CE::execute_waitTicks(count_);
                        state = State::normal;
                        break;
                    case Command::sleep:
                        CE::execute_waitTime(count_);
                        state = State::normal;
                        break;
                }
                break;
            case State::data:
                buff_[ind_++] = c;
                if ( --left_ == 0 ){
                    switch ( cmd ){
                        case Command::scanIo:
                            CE::execute_scanIo(count_,buff_.data());
                            CE::write(buff_.data(),ind_);
                            break;
                        case Command::scanO:
                            CE::execute_scanO(count_,buff_.data());
                            break;
                        case Command::move:
                            CE::execute_move(count_,buff_.data());
                            break;
                        default:
                            break;
                    }
                    state = State::normal;
                }
                break;
        };
    }
    Packets(auto&&... args):CE(std::forward<decltype(args)>(args)...),state(State::normal){}
private:
    std::array<uint8_t,bufLen> buff_;
    uint16_t count_;
    uint16_t left_;
    uint16_t ind_;
    State state;
    Command cmd;
    static constexpr uint8_t i_am_alive_string[] = "djmv1\r\n";
};
}
