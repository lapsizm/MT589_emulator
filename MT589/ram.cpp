#include "ram.h"

RAM::RAM() {
   memory.resize(size);
   for (auto& cell : memory) {
       cell = 0x0000;
   }
}
void RAM::write(size_t address, WORD data) {
    memory[address] = data;
}
WORD RAM::read(size_t address) {
    return memory[address];
}
