#ifndef MK589_HPP
#define MK589_HPP

#include "cpe.h"
#include "mcu.h"
#include "rom.h"
#include "ram.h"



// Microcomputer with microcommand control (by default)
class MK589
{
private:
    HANDLE hSerial;
public:
    MK589();
    MK589(const MK589& mk);
    MK589& operator=(const MK589& mk);

    void SetHSerial(HANDLE& hser){
        hSerial = hser;
    }


    std::vector<CPE> cpe_arr;
    ROM rom {}; // rom with microprogramm
    RAM ram {}; // read/write memory
    MCU mcu {};
    void reset();
    void load(std::bitset<8> x);
    void decode();
    void do_fetch_decode_execute_cycle(const microcommand& mc);
    size_t get_row_adr(); // mcu
    size_t get_col_adr(); // mcu
    //====================INPUTS
    BYTE ED : 1;
    BYTE EA : 1;
    //====================OUTPUTS
    std::optional<BYTE> A; // MAR output
    std::optional<BYTE> D; // AC output
    std::optional<BYTE> RO;
    std::optional<BYTE> CO;
    //====================MEMORY
    WORD MEM[0xC];
    WORD MAR;
private:
    size_t cpe_amount = 8;
    //====================CPE
    //====INPUTS
    std::bitset<7> F;
    WORD K;
    BYTE CI : 1; // only affects lowest 2bits (cpe_arr[0])
    BYTE LI : 1; // only affects highest 2bits (cpe_arr[cpe_amount - 1])
    WORD I; // from IO devices
    WORD M; // from RAM
    //====FUNCS
    void fetch_cpe(std::bitset<7> f,
                   WORD k,
                   WORD i,
                   WORD m,
                   BYTE ed,
                   BYTE ea);
    void decode_cpe();
    void execute_cpe();
    void execute_cpe_right_rot();
    void unite_registers();
    //====decoding
    size_t r_group;
    size_t f_group;
    BYTE ADR : 4;
    bool is_performing_right_rot = false;
    //====================MCU
    //====INPUTS
    BYTE FI : 1; // FI - is CO or RO
    //====OUTPUTS
    BYTE FO : 1; // FO is CI or LI
    //====decoding
    void decode_adr();
    size_t col_adr;
    size_t row_adr;
    //====================
};
#endif // MK589_HPP
