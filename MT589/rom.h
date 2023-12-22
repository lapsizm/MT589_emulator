#ifndef ROM_H
#define ROM_H

#include "mt_global.h"

struct microcommand {
    bool empty = true;

    std::bitset<7> AC;
    BYTE FC : 4;
    std::bitset<7> F;
    WORD K;
    BYTE ED : 1;
    BYTE EA : 1;
    BYTE LD : 1;
    // RAMC
    BYTE CS : 1;
    BYTE RW : 1;
    [[maybe_unused]] WORD M;
    [[maybe_unused]] WORD I;
    //  UI
    int index_F;
    int index_REG;
    int index_FIC;
    int index_FOC;
    int index_Jump;
    std::string address_control = "0000000";

    // For naming
    std::string tag = "";

    // for external stm32:
    uint8_t is_external_command : 1;    // является ли МК - общением с внешним устройством
    std::vector<uint8_t> f_ext;        // отправляемые данные (4 байта)

};

struct ROM
{
    ROM();
    void write(size_t row, size_t col, microcommand cmd);
    microcommand read(size_t row, size_t col);
    bool is_nop(size_t row, size_t col);

    std::vector< std::vector<microcommand> > memory;
    const size_t _rows = 0x20; // 32
    const size_t _cols = 0x10; // 16
};

#endif // ROM_H
