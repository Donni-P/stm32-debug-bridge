#include <flash.h>

namespace flash{
    __attribute__((section(".flash"))) volatile Flash::Regs Flash::regs[1];
}
