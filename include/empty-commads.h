#include <global_resources.h>

namespace commands{
class CommandExecutor{
public:
    void push (char c){
        global::shellRx.pushSafe(c);
    }
    template<size_t L>
    void execute(size_t argc, const std::array<std::string_view, L> &argv){
        for ( size_t i = 0; i < argc ; i++){
            for (uint8_t j:argv[i])
                push(j);
            if ( i!=argc-1)
                push(' ');
        }
        push('\r');
        push('\n');
    }
private:
};
}

