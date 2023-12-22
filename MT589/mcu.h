#ifndef MCU_H
#define MCU_H

#include "mt_global.h"

// workflow :
// 0. fetch(0, x, 0, 1),  exec_next_adr_logic - set start address
// 1. fetch(ac, x, fc) - fetch
// 2. (after cpe execution) fetch_flag(fi);
// 3. decode() -> decode jmp and flag operations
// 4. execute() -> set internal flag registers, set flag output, compute next address (jmp func)

// total pins = 34 (vcc, gnd and clk have no need to be implemented)
// 3 not implemented - ERA, EN, ISE
class MCU
{
public:
    MCU();
    void reset();
    void fetch(std::bitset<7> ac, std::bitset<8> x, BYTE fc, BYTE ld);
    void fetch_flag(BYTE fi); // before flag input logic and next address computations
    void decode();
    void execute_input_flag_logic();
    void execute_output_flag_logic();
    void execute_next_address_logic();
    //======INPUTS
    BYTE FC_10 : 2; // pins 16,15 - 2 total
    BYTE FC_32 : 2; // pins 12,13 - 2 total
    std::bitset<7> AC; // pins 39,38,37,24,23,22,21 - 7 total
    std::bitset<8> X; // pins 1,2,3,4,5,6,8,10 - 8 total
    BYTE FI : 1; // pin 17 - 1 total
    BYTE LD : 1; // pin 36 - 1 total
    //ERA pin 35 - 1 total
    //EN pin 25 - 1 total
    //======OUTPUTS
    //ISE pin 18 - 1 total
    BYTE FO : 1; // pin 14 - 1 total
    BYTE PR : 3; // pins 7,9,11 - 3 total
    std::bitset<9> MA; // pins 26,27,28,29,30,31,32,33,34 - 9 total
    //======MEMORY
    BYTE TF : 1; // flag latch
    BYTE TC : 1; // CO trigger (lathc)
    BYTE TZ : 1; // ZERO flag latch (trigger)
    std::bitset<4> PR_latch; // aka command register
    std::bitset<9> MPAR; // microprogramm address register
    //======MISC
    void decode_jmp();
    void decode_fl(); // decode flag logic
    enum class FCI : BYTE {
        SCZ = 0b00,
        STZ = 0b01,
        STC = 0b10,
        HCZ = 0b11
    };
    enum class FCO : BYTE{
        FF0 = 0b00,
        FFC = 0b01,
        FFZ = 0b10,
        FF1 = 0b11
    };
    enum class JUMP {
        JCC = 0,
        JZR,
        JCR,
        JCE,
        JFL,
        JCF,
        JZF,
        JPR,
        JLL,
        JRL,
        JPX
    };
    static constexpr size_t total_jmps = 11;
    BYTE _signals[total_jmps] = {
        0b0011111, // JCC
        0b0101111, // JZR
        0b0111111, // JCR
        0b1110111, // JCE
        0b1001111, // JFL
        0b1010111, // JCF
        0b1011111, //
        0b1100111,
        0b1101111,
        0b1111111,
        0b1111011
    };
    JUMP cur_jmp;
    FCI cur_FCI;
    FCO cur_FCO;
};

#endif // MCU_H
