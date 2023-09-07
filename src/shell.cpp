#include <device-shell.h>
#include <shell.h>
#include COMMANDS_HEADER

namespace deviceShell{
void tick (uint8_t c){
    const static char prompt[] = "> ";
    static shell::Shell<
        commands::CommandExecutor,
        prompt,
        60,
        8,
        shell::color::index::green,
        false,
        false,
        false,
        16
    > sh;
    sh.exec(c);
}
}
