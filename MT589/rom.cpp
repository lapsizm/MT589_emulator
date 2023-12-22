#include "rom.h"

ROM::ROM() {
   memory.resize(_rows);
   for(auto& row : memory) {
       row.resize(_cols);
   }
   for (auto& row : memory) {
       for (auto& cell : row) {
           cell.F = 0b1100000;
           cell.K = 0b00000000;
           cell.FC = 0b0011;
           cell.AC = 0b0000000;
           cell.ED = 0;
           cell.EA = 0;
           cell.LD = 0;
           cell.RW = 0;
           cell.CS = 0;
           cell.is_external_command = 0;
           cell.f_ext.resize(4);
           cell.f_ext = {0,0,0,0};
       }
   }
}

bool ROM::is_nop(size_t row, size_t col) {
    return memory[row][col].empty;
}

void ROM::write(size_t row, size_t col, microcommand cmd) {
    memory[row][col] = cmd;
}

microcommand ROM::read(size_t row, size_t col) {
    return memory[row][col];
}
