#include "emulator.hpp"
#include <iostream>


void visit(std::vector< std::vector<bool> >& rom, size_t row, size_t col) {
    rom[row][col] = true;
}
void print(const std::vector< std::vector<bool> >& rom) {
    for (size_t i = 0; i < 32; ++i) {
        for (size_t j = 0; j < 16; ++j) {
            std::cout << (rom[i][j] ? "+" : "-")<< " ";
        }
        std::cout << std::endl;
    }

}

int main() {
    MCU mce {};
    std::bitset<8> X = 0b00010001;
    std::bitset<7> AC;
    mce.X = X;
    mce.load(X);

    std::vector<std::vector<bool> > rom;
    std::vector< std::bitset<7> > prog = {
        {0b0110010},
        {0b0110011},
        {0b0000010},
        {0b0000011},
        {0b0000100},
    };

    rom.resize(32);
    for (auto& i : rom) {
        i.resize(16);
    }
    for (size_t i = 0; i < 32; ++i) {
        for (size_t j = 0; j < 16; ++j) {
            rom[i][j] = false;
        }
    }
    print(rom);
    for (const auto& p : prog) {
        mce.connect_data(p, X, 0b1, 0b0000);
        //mce.decode_jmp();
        //std::cout << static_cast<int>(mce.cur_jmp) << std::endl;
        //mce.compute_next_addr();
        //auto ma = mce.MA;
        //std::cout << ma << std::endl;;
        mce.execute();
        std::cout << mce.row_adr << " " << mce.col_adr << std::endl;
        visit(rom, mce.row_adr, mce.col_adr);
    }

    print(rom);
}
