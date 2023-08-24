#pragma once

#include <utility>
#include <cstdint>
#include <string_view>
#include <array>

namespace shell{

namespace color{
enum class index{
    red,
    green,
    yellow,
    blue,
    purple,
    magenta,
    white,
    normal
};
constexpr std::string_view get(index ind){
    using namespace std::literals::string_view_literals;
    switch (ind){
        case index::red:
            return "\033[31m"sv;
        case index::green:
            return "\033[32m"sv;
        case index::yellow:
            return "\033[33m"sv;
        case index::blue:
            return "\033[34m"sv;
        case index::purple:
            return "\033[35m"sv;
        case index::magenta:
            return "\033[36m"sv;
        case index::white:
            return "\033[37m"sv;
        case index::normal:
            return "\033[0m"sv;
    };
    return "\033[0m"sv;
}
}

namespace endl{
enum class index{
    _r,
    _n,
    _r_n,
    _n_r
};
constexpr std::string_view get(index ind){
    using namespace std::literals::string_view_literals;
    switch(ind){
        case index::_r:
            return "\r"sv;
        case index::_n:
            return "\n"sv;
        case index::_r_n:
            return "\r\n"sv;
        case index::_n_r:
            return "\n\r"sv;
    }
    return "\r\n"sv;
}
}

template <
    typename CE,
    const char * prompt,
    std::size_t lineLen = 60,
    std::size_t tokNum = 8,
    color::index promptColor = color::index::green,
    bool useComplete = false,
    bool useQuoting = false,
    bool useEchoOff = false,
    size_t historyLen = 0,
    bool useEscSeq = true,
    bool useCarriageReturn = true,
    bool useCtrlC = false,
    bool promptOnInit = true,
    endl::index endLine = endl::index::_r_n
>
    requires requires(CE ce, char c, std::array<std::string_view, tokNum> l){
        ce.execute(tokNum, l);
        ce.push(c);
    }
class Shell:public CE{
public:

    enum class res{
        ok,
        err,
        errPar,
        errTokNum,
        errClFull,
        errCplt
    };
    enum class seq{
        none,
        esc,
        csi,
        home,
        end,
        del,
        seq4f
    };
    enum class echo {
        once,
        on,
        off
    };
    struct escAnsi {
        static constexpr char nul = 0x00;
        static constexpr char soh = 0x01;
        static constexpr char stx = 0x02;
        static constexpr char etx = 0x03;
        static constexpr char eot = 0x04;
        static constexpr char neq = 0x05;
        static constexpr char ack = 0x06;
        static constexpr char bel = 0x07;
        static constexpr char bs  = 0x08;
        static constexpr char ht  = 0x09;
        static constexpr char lf  = 0x0a;
        static constexpr char vt  = 0x0b;
        static constexpr char ff  = 0x0c;
        static constexpr char cr  = 0x0d;
        static constexpr char so  = 0x0e;
        static constexpr char si  = 0x0f;
        static constexpr char dle = 0x10;
        static constexpr char dc1 = 0x11;
        static constexpr char dc2 = 0x12;
        static constexpr char dc3 = 0x13;
        static constexpr char dc4 = 0x14;
        static constexpr char nak = 0x15;
        static constexpr char syn = 0x16;
        static constexpr char etb = 0x17;
        static constexpr char can = 0x18;
        static constexpr char em  = 0x19;
        static constexpr char sub = 0x1a;
        static constexpr char esc = 0x1b;
        static constexpr char fs  = 0x1c;
        static constexpr char gs  = 0x1d;
        static constexpr char rs  = 0x1e;
        static constexpr char us  = 0x1f;

        static constexpr char del = 0x7f;
        static constexpr char csi = '\233';
    };

    static size_t tokenize(std::string_view line, std::array<std::string_view, tokNum> &toks)
        requires (useQuoting == false){
        enum stat{
            empty,
            token

        }status = empty;
        size_t tokInd = 0;
        size_t tokBegin = 0;
        using namespace std::literals;
        using std::string_view;
        static const string_view separators = " \t\n\f\r\v"sv;
        for ( size_t i = 0 ; i < line.size() ; ++i ){
            if ( separators.find(line[i])!=string_view::npos ){
                if (status == token){
                    if ( tokInd == tokNum ) throw res::errTokNum;
                    toks[tokInd++] = line.substr(tokBegin, i-tokBegin);
                    status = empty;
                }
            }else{
                if (status == empty){
                    tokBegin = i;
                    status = token;
                }
            }
        }

        if ( status == token ){
            if ( tokInd == tokNum ) throw res::errTokNum;
            toks[tokInd++] = line.substr(tokBegin);
            status = empty;
        }

        return tokInd;
    }
    static size_t tokenize(std::string_view line, std::array<char, lineLen> &buf, std::array<std::string_view, tokNum> &toks)
        requires (useQuoting == true){
        enum stat{
            empty,
            token,
            singleQuotes,
            doubleQuotes,
            tokenEsc,
            singleQuotesEsc,
            doubleQuotesEsc

        }status = empty;
        size_t tokInd = 0;
        size_t tokBegin = 0;
        size_t bufInd = 0;
        for ( size_t i = 0 ; i < line.size() ; ++i ){
            switch ( line[i] ){
                case '\0':
                    break;
                case '\t':
                case ' ':{
                        switch ( status ){
                            case empty:
                                break;
                            case token:
                                toks[tokInd++] = std::string_view(buf.data()+tokBegin,bufInd-tokBegin);
                                status = empty; 
                                break;
                            case tokenEsc:
                                status = token;
                                [[fallthrough]];
                            case singleQuotes:
                            case doubleQuotes:
                                buf[bufInd++] = line[i];
                                break;
                            case singleQuotesEsc:
                                status = singleQuotes;
                                buf[bufInd++] = line[i];
                                break;
                            case doubleQuotesEsc:
                                status = doubleQuotes;
                                buf[bufInd++] = line[i];
                                break;
                        }
                    }
                    break;
                case '\\':{
                        switch ( status ){
                            case empty:
                                if ( tokInd == tokNum ) throw res::errTokNum;
                                tokBegin = bufInd;
                                [[fallthrough]];
                            case token:
                                status = tokenEsc;
                                break;
                            case singleQuotes:
                                status = singleQuotesEsc;
                                break;
                            case doubleQuotes:
                                status = doubleQuotesEsc;
                                break;
                            case tokenEsc:
                                status = token;
                                buf[tokInd++] = '\\';
                                break;
                            case singleQuotesEsc:
                                status = singleQuotes;
                                buf[tokInd++] = '\\';
                                break;
                            case doubleQuotesEsc:
                                status = doubleQuotes;
                                buf[tokInd++] = '\\';
                                break;
                        }
                    }
                    break;
                case '\'':{
                        switch ( status ){
                            case empty:
                                if ( tokInd == tokNum ) throw res::errTokNum;
                                status = singleQuotes;
                                break;
                            case token:
                                status = singleQuotes;
                                break;
                            case doubleQuotesEsc:
                                buf[bufInd++] = '\'';
                                status = doubleQuotes;
                                break;
                            case singleQuotes:
                                status = token;
                                break;
                            case doubleQuotes:
                            case tokenEsc:
                            case singleQuotesEsc:
                                buf[bufInd++] = '\'';
                                status = singleQuotes;
                                break;
                        }
                    }
                    break;
                case '\"':{
                        switch ( status ){
                            case empty:
                                if ( tokInd == tokNum ) throw res::errTokNum;
                                status = doubleQuotes;
                                break;
                            case token:
                                status = doubleQuotes;
                                break;
                            case singleQuotesEsc:
                                buf[bufInd++] = '\"';
                                break;
                            case doubleQuotes:
                                status = token;
                                break;
                            case singleQuotes:
                            case tokenEsc:
                            case doubleQuotesEsc:
                                buf[bufInd++] = '\"';
                                break;
                        }
                    }
                    break;
                default:{
                        switch ( status ) {
                            case empty:
                                if ( tokInd == tokNum ) throw res::errTokNum;
                                status = token;
                                tokBegin = bufInd;
                                [[fallthrough]];
                            case token:
                            case singleQuotes:
                            case doubleQuotes:
                                buf[bufInd++] = line[i];
                                break;
                            case tokenEsc:
                            case singleQuotesEsc:
                            case doubleQuotesEsc:
                                throw res::err;
                                break;
                        }
                    }
                    break;
            }
        }
        switch ( status ){
            case empty:
                break;
            case token:
                toks[tokInd++] = std::string_view(buf.data()+tokBegin,bufInd-tokBegin);
                status = empty; 
                break;
            case singleQuotes:
            case doubleQuotes:
            case tokenEsc:
            case singleQuotesEsc:
            case doubleQuotesEsc:
                throw res::err;
                break;
        }
        return tokInd;
    }

    static inline bool paste ( std::array<char, lineLen> &dst, std::size_t len, std::size_t pos, std::string_view text ){
        if ( len + text.size() > lineLen ){
            return false;
        }
        if ( pos > len ){
            return false;
        }
        for ( size_t i = 0 ; i < len-pos ; i++ ){
            dst[len+text.size()-i-1] = dst[len-i-1];
        }
        for ( size_t i = 0 ; i < text.size() ; i++ ){
            dst[pos+i] = text[i];
        }
        return true;
    }

    template<typename... Args>
    constexpr Shell(Args... args):CE(args...),line_(),pos(0),historyInd(0),historyDeep(0),echoState(echo::on),seqState(seq::none){
        for ( auto &i:len ){
            i = 0;
        }
    }

    void exec(char c) requires(useEscSeq=false){
        processCharNormal(c);
    }

    void exec(char c) requires(useEscSeq==true){
        switch ( seqState ){
            case seq::none:
                processCharNormal(c);
                break;
            case seq::esc:
                processCharEsc(c);
                break;
            case seq::csi:
                processCharCsi(c);
                break;
            case seq::home:
                processCharHome(c);
                break;
            case seq::end:
                processCharEnd(c);
                break;
            case seq::del:
                processCharDel(c);
                break;
            case seq::seq4f:
                processCharSeq4f(c);
                break;
        }
    }

    void setEchoState (echo v) requires(useEchoOff==true){
        echoState = v;
    }

    echo getEchoState (void) requires(useEchoOff==true){
        return echoState;
    }
private:

    size_t tokenize () requires (useQuoting == true){
        return tokenize(std::string_view(line_[historyLen].data(),len[historyLen]),line_[historyLen+1],tokens);
    }
    size_t tokenize () requires (useQuoting == false){
        return tokenize(std::string_view(line_[historyLen].data(),len[historyLen]),tokens);
    }
    size_t deleteText(size_t count){
        if ( count > pos ){
            count = pos;
        }
        moveCursor(-static_cast<int>(count));
        for ( size_t i = 0; i < len[historyLen] - pos ; i++ ){
            char c=line_[historyLen][pos+i];
            pushEcho(c);
            line_[historyLen][pos-count+i]=c;
        }
        for ( size_t i = 0 ; i < count ; i++ )
            pushEcho(' ');
        moveCursor(-static_cast<int>(len[historyLen]-pos+count));
        pos -= count;
        len[historyLen] -= count;
        return count;
    }
    bool paste(std::string_view text){
        bool res = paste ( line_[historyLen], len[historyLen], pos, text );
        if ( res ){
            for ( size_t i = 0; i < len[historyLen] + text.size() - pos ; i++ )
                pushEcho ( line_[historyLen][pos+i] );
            moveCursor(-static_cast<int>(len[historyLen]-pos));
            pos += text.size();
            len[historyLen] += text.size();
        }
        return res;
    }
    void moveCursor (int count){
        char dir = 'C';
        if ( count == 0 ) {}
        else if ( count == -1 ){
            pushEcho('\b');
            return;
        }else if ( count == 1 ){
            pushEcho('\033');
            pushEcho('[');
            pushEcho('C');
        }else{
            if ( count < 0 ){
                dir = 'D';
                count = (-count);
            }
            if ( count > 999 )
                count = 999;
            pos -= count;
            pushEcho('\033');
            pushEcho('[');
            if ( count > 100 )
                pushEcho(count/100%10+'0');
            if ( count > 10 )
                pushEcho(count/10%10+'0');
            pushEcho(count%10+'0');
            pushEcho(dir);
        }
    }
    void processCharNormal (char c){
        switch ( c ){
            case '\b':
            case escAnsi::del:
                deleteText(1);
                break;
            case '\n':
            case '\r':
                {
                    if ( useEchoOff && echoState == echo::once )
                        echoState = echo::on;
                    for ( auto i: endl_ )
                        pushEcho(i);
                    std::size_t num = tokenize();
                    if ( num > 0 ){
                        CE::execute(num,tokens);
                        if ( historyLen > 0 ){
                            line_[historyInd] = line_[historyLen];
                            len[historyInd] = len[historyLen];
                            historyInd = (historyInd + 1) % historyLen;
                        }
                    }
                    for ( auto i: colorPmt_ )
                        pushEcho(i);
                    for ( auto i: prompt_ )
                        pushEcho(i);
                    for ( auto i: colorDef_ )
                        pushEcho(i);
                    len[historyLen] = 0;
                    pos = 0;
                    historyDeep = 0;
                    break;
                }
            case escAnsi::esc:
                if ( useEscSeq )
                    seqState = seq::esc;
                break;
            case escAnsi::csi:
                if ( useEscSeq )
                    seqState = seq::csi;
                break;
            default:
                paste(std::string_view(&c,1));
                break;
        }
    }
    void processCharEsc(char c) requires (useEscSeq == true){
        switch ( c ){
            case '[':
                seqState = seq::csi;
                break;
            case 'O':
                seqState = seq::seq4f;
                break;
            default:
                seqState = seq::none;
                break;

        }
    }
    void processCharCsi(char c) requires (useEscSeq == true){
        seqState = seq::none;
        switch ( c ){
            case 'A':
                if ( historyLen>0 ){
                    if ( historyDeep == 0 ){
                        line_[historyLen+1] = line_[historyLen];
                        len[historyLen+1] = len[historyLen];
                    }
                    if ( historyDeep < historyLen && len[(historyLen+historyInd-historyDeep-1)%historyLen] > 0 ){
                        line_[historyLen] = line_[(historyLen+historyInd-historyDeep-1)%historyLen];
                        len[historyLen] = len[(historyLen+historyInd-historyDeep-1)%historyLen];
                        historyDeep++;
                        pos = len[historyLen];
                    }
                }
                rePrint ();
                break;
            case 'B':
                if ( historyLen > 0 ){
                    if ( historyDeep > 0 ){
                        historyDeep--;
                        if ( historyDeep == 0 ){
                            line_[historyLen] = line_[historyLen+1];
                            len[historyLen] = len[historyLen+1];
                        }else{
                            line_[historyLen] = line_[(historyLen+historyInd-historyDeep-1)%historyLen];
                            len[historyLen] = len[(historyLen+historyInd-historyDeep-1)%historyLen];
                        }
                        pos = len[historyLen];
                    }
                }
                rePrint ();
                break;
            case 'C':
                if ( pos < len[historyLen] ){
                    pos++;
                    moveCursor(1);
                }
                break;
            case 'D':
                if ( pos > 0 ){
                    pos--;
                    moveCursor(-1);
                }
                break;
            case '1':
            case '7':
                seqState = seq::home;
                break;
            case '4':
            case '8':
                seqState = seq::end;
                break;
            case '3':
                seqState = seq::del;
                break;
        }
    }
    void processCharHome(char c) requires (useEscSeq == true){
        seqState = seq::none;
        if ( c == '~' ){
            moveCursor(-static_cast<int>(pos));
            pos = 0;
        }
    }
    void processCharEnd(char c) requires (useEscSeq == true){
        seqState = seq::none;
        if ( c == '~' ){
            moveCursor(len[historyLen]-pos);
            pos = len[historyLen];
        }
    }
    void processCharDel(char c) requires (useEscSeq == true){
        seqState = seq::none;
        if ( c == '~' ){
            if ( pos < len[historyLen] ){
                pos++;
                deleteText(1);
            }
        }
    }
    void processCharSeq4f(char c) requires (useEscSeq == true){
        seqState = seq::none;
        switch(c){
            case 'F':
                moveCursor(len[historyLen]-pos);
                pos = len[historyLen];
                break;
            default:
                break;
        }
    }
    void pushEcho (char c) requires(useEchoOff==false){
        CE::push(c);
    }
    void pushEcho (char c) requires(useEchoOff==true){
        if ( echoState == echo::on )
            CE::push(c);
    }
    void rePrint (void){
        int curPos = pos;
        if ( useCarriageReturn ){
            pushEcho('\r');
        }else{
            moveCursor (-curPos-static_cast<int>(prompt_.size()));
        }
        pushEcho('\033');
        pushEcho('[');
        pushEcho('K');
        for ( auto i : prompt_ ){
            pushEcho (i);
        }
        for ( size_t i = 0 ; i < len[historyLen] ; i++ ){
            pushEcho(line_[historyLen][i]);
        }
        moveCursor ( pos-static_cast<int>(len[historyLen]) );
    }
    static constexpr uint32_t buffCount = historyLen+((useQuoting||(historyLen>0))?2:1);
    std::array<std::array<char,lineLen>,buffCount> line_;
    std::array<std::string_view, tokNum> tokens;
    static constexpr std::string_view colorPmt_ = color::get(promptColor);
    static constexpr std::string_view colorDef_ = color::get(color::index::normal);
    static constexpr std::string_view endl_ = endl::get(endLine);
    static constexpr std::string_view prompt_ = prompt;
    std::array<std::size_t, buffCount> len;
    std::size_t pos;
    std::size_t historyInd;
    std::size_t historyDeep;
    echo echoState;
    seq seqState;
};

} // namespace shell
