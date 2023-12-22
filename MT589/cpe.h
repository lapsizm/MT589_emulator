#ifndef CPE_H
#define CPE_H

#include "mt_global.h"
#include <windows.h>

// total pins = 25 (vcc, gnd and clk have no need to be implemented)
class CPE
{

public:
    CPE();
    void fetch(const std::bitset<7>& f, BYTE i, BYTE k, BYTE m, BYTE CI,
               BYTE LI, BYTE ed, BYTE ea);
    void decode();
    void execute();
    void reset();
    //======INPUT
    std::bitset<7> F; // pins 25,26,27,24,17,16,15 - 7 total
    BYTE I : 2; // pins 1,2 - 2 total
    BYTE K : 2; // pins 3,4 - 2 total
    BYTE CI : 1; // pin 10 - 1 total
    BYTE LI : 1; // pin 9 - 1 total
    BYTE M : 2; // pins 22, 21 - 2 total
    BYTE EA : 1; // pin 11 - 1 total
    BYTE ED : 1; // pin 23 - 1 total
    //======OUTPUT
    std::optional<BYTE> A; // pins 12,13 - 2 total
    std::optional<BYTE> D; // pins 20,19 - 2 total
    std::optional<BYTE> RO; // pin 8 - 1 total
    std::optional<BYTE> CO; // pin 7 - 1 total
    BYTE X : 1; // pin 5 - 1 total
    BYTE Y : 1; // pin 6 - 1 total
    //======MEMORY
    BYTE MAR : 2;
    BYTE MEM[0xC];

    // X, Y and CO output computations
    void propogate();
    void compute_CO();
    // CPE f-group functions
    void execute_f0();
    void execute_f1();
    void execute_f2();
    void execute_f3();
    void execute_f4();
    void execute_f5();
    void execute_f6();
    void execute_f7();

    /* заглушка для общения с внешним устр-вом */
    void execute_f8();

    //======MISC
    BYTE word_wise_or(BYTE op);
    BYTE get_lb(BYTE src);
    BYTE get_hb(BYTE src);
    int r_group;
    int f_group;
    BYTE ADR : 4;
    BYTE BUF1 : 1;
    BYTE BUF2 : 2;
    BYTE opA : 2;
    BYTE opB : 2;
};

#endif // CPE_H
