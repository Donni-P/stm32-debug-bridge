#include <cstdint>
namespace flash{
class Flash{
public:
    struct Regs{
        static_assert (sizeof(uint32_t) == sizeof(const void *));
        std::uint32_t ACR;
        std::uint32_t KEYR;
        std::uint32_t OPTKEYR;
        std::uint32_t SR;
        std::uint32_t CR;
        const void * AR;
        std::uint32_t reserved;
        std::uint32_t OBR;
        std::uint32_t WRPR;
    };
    volatile Regs * operator -> () {return regs;}
    struct ACR{
        static constexpr std::uint32_t prftbsInd = 5;
        static constexpr std::uint32_t prftbsMsk = 1u << prftbsInd;

        static constexpr std::uint32_t prftbeInd = 4;
        static constexpr std::uint32_t prftbeMsk = 1u << prftbeInd;

        static constexpr std::uint32_t hlfcyaInd = 3;
        static constexpr std::uint32_t hlfcyaMsk = 1u << hlfcyaInd;

        static constexpr std::uint32_t latencyMsk  = 7u;
        static constexpr std::uint32_t latencyZero = 0u;
        static constexpr std::uint32_t latencyOne  = 1u;
        static constexpr std::uint32_t latencyTwo  = 2u;

        static constexpr std::uint32_t reservedMsk = ~(prftbsMsk|prftbeMsk|hlfcyaInd|latencyMsk);
    };
    struct KEYR{
        static constexpr std::uint32_t rdprt = 0x00A5;
        static constexpr std::uint32_t key1 = 0x45670123;
        static constexpr std::uint32_t key2 = 0xCDEF89AB;
    };
    struct SR{
        static constexpr std::uint32_t eopInd = 5;
        static constexpr std::uint32_t eopMsk = 1u << eopInd;

        static constexpr std::uint32_t wrPrtErrInd = 4;
        static constexpr std::uint32_t wrPrtErrMsk = 1u << wrPrtErrInd;

        static constexpr std::uint32_t pgErrInd = 2;
        static constexpr std::uint32_t pgErrMsk = 1u << pgErrInd;

        static constexpr std::uint32_t bsyMsk = 1u;
    };
    struct CR{
        static constexpr std::uint32_t eopIeInd = 12;
        static constexpr std::uint32_t eopIeMsk = 1u << eopIeInd;

        static constexpr std::uint32_t errIeInd = 10;
        static constexpr std::uint32_t errIeMsk = 1u << errIeInd;

        static constexpr std::uint32_t optWrEInd = 9;
        static constexpr std::uint32_t optWrEMsk = 1u << optWrEInd;

        static constexpr std::uint32_t lockInd = 7;
        static constexpr std::uint32_t lockMsk = 1u << lockInd;

        static constexpr std::uint32_t strtInd = 6;
        static constexpr std::uint32_t strtMsk = 1u << strtInd;

        static constexpr std::uint32_t optErInd = 5;
        static constexpr std::uint32_t optErMsk = 1u << optErInd;

        static constexpr std::uint32_t optPgInd = 4;
        static constexpr std::uint32_t optPgMsk = 1u << optPgInd;

        static constexpr std::uint32_t mErInd = 2;
        static constexpr std::uint32_t mErMsk = 1u << mErInd;

        static constexpr std::uint32_t pErInd = 1;
        static constexpr std::uint32_t pErMsk = 1u << pErInd;

        static constexpr std::uint32_t pgInd = 0;
        static constexpr std::uint32_t pgMsk = 1u << pgInd;

        static constexpr std::uint32_t knownMsk = eopIeMsk | errIeMsk | optWrEMsk | lockMsk | strtMsk | optErMsk | optPgMsk | mErMsk | pErMsk | pgMsk;

    };
    static void setLatency (uint32_t l){
        uint32_t v = ( regs->ACR & ~(ACR::hlfcyaMsk | ACR::latencyMsk) ) | ACR::prftbeMsk;
        switch (l){
        case 0:
            v |= ACR::latencyZero;
        break;
        case 1:
            v |= ACR::latencyOne;
        break;
        case 2:
        default:
            v |= ACR::latencyTwo;
        break;
        }
        regs->ACR = v;
    }
    static bool isLock(void){
        return ( regs->CR & CR::lockMsk ) == CR::lockMsk;
    }
    static bool unlock (void){
        if ( isLock() ){
            regs->KEYR = KEYR::key1;
            regs->KEYR = KEYR::key2;
            for ( unsigned i = 0 ; i < 16 ; i++ ){
                if ( !isLock() )
                    return true;
            }
        }
        return false;
    }
    static void lock (void){
        uint32_t cr = regs->CR & CR::knownMsk;
        if ( !isLock() ){
            regs->CR = cr | CR::lockMsk; 
        }
    }
    static void wait (void){
        while ( ( ( regs->CR & CR::strtMsk ) == CR::strtMsk ) ||
                ( ( regs->SR & SR::bsyMsk ) == SR::bsyMsk ) );
    }
    static void waitEop (void){
        while ( ( regs->SR & SR::eopMsk ) == 0 );
        regs->SR = SR::eopMsk;
    }
    static bool massErase (void){
        if ( !isLock() ){
            wait();
            std::uint32_t cr = regs->CR & CR::knownMsk;
            regs->CR = cr | CR::mErMsk;
            regs->CR = cr | CR::mErMsk | CR::strtMsk;
            waitEop();
            regs->CR = cr;
            return true;
        }else{
            return false;
        }
    }
    static bool pageErase (void * addr){
        if ( !isLock() ){
            wait();
            std::uint32_t cr = regs->CR & CR::knownMsk;
            regs->CR = cr | CR::pErMsk;
            regs->AR = addr;
            regs->CR = cr | CR::pErMsk | CR::strtMsk;
            waitEop();
            regs->CR = cr;
            return true;

        }else{
            return false;
        }
    }
    static bool write ( volatile uint16_t *addr, uint16_t data ){
        if ( !isLock() ){
            wait();
            std::uint32_t cr = regs->CR & CR::knownMsk;
            regs->CR = cr | CR::pgMsk;
            *addr = data;
            waitEop();
            regs->CR = cr;
            return true;

        }else{
            return false;
        }
    }

private:
    static volatile Regs regs[1];
};
}
