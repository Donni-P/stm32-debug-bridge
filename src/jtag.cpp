
#include <jtag.h>

#include <global_resources.h>

namespace jtag {
void tick(char c) {
    const uint8_t i_am_alive_string[] = "I'm alive!\r\n";
    switch (c) {
    case 'B':
        config::ledOn();
        break;
    case 'b':
        config::ledOff();
        break;
    case 'R':
        global::jtagRx.push( global::jtagIn.read() ? '1' : '0');
        break;
    case '0':
            global::jtagOut.write(false, false, false);
        break;
    case '1':
            global::jtagOut.write(true, false, false);
        break;
    case '2':
            global::jtagOut.write(false, true, false);
        break;
    case '3':
            global::jtagOut.write(true, true, false);
        break;
    case '4':
            global::jtagOut.write(false, false, true);
        break;
    case '5':
            global::jtagOut.write(true, false, true);
        break;
    case '6':
            global::jtagOut.write(false, true, true);
        break;
    case '7':
            global::jtagOut.write(true, true, true);
        break;
    case 'r':
    case 't':
        global::jtagOut.write<2>(true);
        break;

    case 's':
    case 'u':
        global::jtagOut.write<2>(false);
        break;
    case 'h':
        global::jtagRx.write(i_am_alive_string,
                             sizeof(i_am_alive_string) - 1);
        break;

    default:
        break;
    }
}
} // namespace jtag
