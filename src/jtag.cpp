
#include <jtag.h>
#include <djm.h>

#include <global_resources.h>

namespace jtag {
class Executor{
public:
    void execute_B(void){
        config::ledOn();
    }
    void execute_b(void){
        config::ledOff();
    }
    void execute_R(void){
        global::jtagRx.push( global::jtagIn.read() ? '1' : '0');
    }
    void execute_0(void){
        global::jtagOut.write(false, false, false);
    }
    void execute_1(void){
        global::jtagOut.write(true, false, false);
    }
    void execute_2(void){
        global::jtagOut.write(false, true, false);
    }
    void execute_3(void){
        global::jtagOut.write(true, true, false);
    }
    void execute_4(void){
        global::jtagOut.write(false, false, true);
    }
    void execute_5(void){
        global::jtagOut.write(true, false, true);
    }
    void execute_6(void){
        global::jtagOut.write(false, true, true);
    }
    void execute_7(void){
        global::jtagOut.write(true, true, true);
    }
    void execute_r(void){}
    void execute_t(void){}
    void execute_s(void){}
    void execute_u(void){}
    void execute_scanIo(uint16_t count, uint8_t* buffer){
        scan<true,true>(count,buffer);
    }
    void execute_scanI(uint16_t count, uint8_t* buffer){
        scan<true,false>(count,buffer);
    }
    void execute_scanO(uint16_t count, uint8_t* buffer){
        scan<false,true>(count,buffer);
    }
    void execute_move(uint16_t count, uint8_t* buffer){
        for ( uint32_t i = 0 ; i < count ; i++ ){
            bool tms = (buffer[i/8]&(1<<(i%8)))?true:false;
            global::jtagOut.write(false, tms, false);
            global::jtagOut.write<2>(true);
        }
        global::jtagOut.write<2>(false);
    }
    void execute_waitTicks(uint16_t count){
        while ( count-- ){
            global::jtagOut.write<2>(false);
            global::jtagOut.write<2>(true);
        }
        global::jtagOut.write<2>(false);
    }
    void execute_waitTime([[maybe_unused]] uint16_t count){}
    void execute_setSpeed([[maybe_unused]] uint16_t count){
    }
    void write(const uint8_t *buf, uint16_t count){
        while ( count-- )
            global::jtagRx.push(*buf++);
    }
    template <bool read, bool write>
    void scan(uint16_t count, uint8_t* buffer){
        uint8_t r = 0;
        for ( uint32_t i = 0 ; i < count ; i++ ){
            if ( ( i%8 == 0 ) && ( i/8 > 0 ) ){
                buffer[i/8-1] = r;
                if (read)
                    r=0;
            }
            bool tdi = write?(buffer[i/8]&(1<<(i%8)))?true:false:false;
            bool tms = (i==count-1u);
            global::jtagOut.write(tdi,tms,false);
            global::jtagOut.write<2>(true);
            if (read)
                r |= global::jtagIn.read()?1<<i%8:0;
        }
        buffer[(count-1)/8] = r;
        global::jtagOut.write(false,false,false);
        global::jtagOut.write<2>(true);
    }
};
void tick(char c) {
    static djm::Packets<Executor> executor;
    executor.exec(c);
}
} // namespace jtag
