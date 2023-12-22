#include <iostream>

#include "emulator.hpp"

void print_registers(const MK589& mk) {
    for (size_t i = 0; i < 0xA; ++i) {
        std::cout << i << " = " << (unsigned)mk.MEM[i] << ", " << std::bitset<8>(mk.MEM[i]) << std::endl;
    }
    std::cout << "AC = " << (unsigned)mk.MEM[AC] << ", " << std::bitset<8>(mk.MEM[AC]) << std::endl;
    std::cout << "T = " << (unsigned)mk.MEM[T] << std::endl;
    std::cout << "MAR = " << (unsigned)mk.MAR << std::endl;

}
void print_out(const CPE& cpe) {

}

int main() {
   MK589 mk {};

   // test for inc of reg
   if (true) {
       std::cout << "============================ILR==========================" << std::endl;
       for (size_t i = 0; i < 256; ++i) {
           mk.fetch_cpe(0b0000000,0b00000000,0b00000000,0b00000000);
	   mk.CI = 1;
           mk.decode_cpe();
           mk.execute_cpe();
	   mk.CO = mk.FI;
           print_registers(mk);
           std::cout << "cpe_arr CO = " << (unsigned)mk.CO << std::endl;
       }
       std::cout << "=========================================================" << std::endl;
   }
   // test for shifting (ALR left, SRA right)
   // if (false) {
   //     std::cout << "============================ALR==========================" << std::endl;
   //     // add 1 to reg0 and AC
   //     mk.fetch_cpe(0b0000000,0b00000000,0b00000000,0b00000000);
   //     mk.decode_cpe();
   //     mk.execute_cpe();
   //     print_registers(mk);
   //     for (size_t i = 0; i < 8; ++i) {
   //         mk.fetch_cpe(0b0001101,0b11111111,0b00000000,0b00000000);
   //         mk.decode_cpe();
   //         mk.execute_cpe();
   //         print_registers(mk);
   //         std::cout << "cpe_arr CO = " << (unsigned)mk.CO << std::endl;
   //     }
   //     std::cout << "=========================================================" << std::endl;
   // }
   // if (false) {
   //     std::cout << "============================SRA==========================" << std::endl;
   //     // LDM : Load M to AT
   //     mk.fetch_cpe(0b0011011,0b00000000,0b00000000,0b10000000);
   //     mk.decode_cpe();
   //     mk.execute_cpe();
   //     print_registers(mk);
   //     for (size_t i = 0; i < 8; ++i) {
   //         mk.fetch_cpe(0b0001111,0b00000000,0b00000000,0b00000000);
   //         mk.decode_cpe();
   //         mk.execute_cpe_right_rot();
   //         print_registers(mk);
   //         std::cout << "cpe_arr RO = " << (unsigned)mk.RO << std::endl;
   //     }
   //     std::cout << "=========================================================" << std::endl;
   // }
   // CPE cpe{};
   // // test of get_hb and get_lb
   // if (false) {
   //     struct test {
   //         BYTE B : 2;
   //     };
   //     test t;
   //     t.B = 0b11;
   //     std::cout << std::bitset<2>(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_hb(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_lb(t.B) << std::endl;
   //     t.B = 0b10;
   //     std::cout << std::bitset<2>(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_hb(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_lb(t.B) << std::endl;
   //     t.B = 0b01;
   //     std::cout << std::bitset<2>(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_hb(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_lb(t.B) << std::endl;
   //     t.B = 0b00;
   //     std::cout << std::bitset<2>(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_hb(t.B) << std::endl;
   //     std::cout << (unsigned)cpe.get_lb(t.B) << std::endl;
   // }
   // // test of SRA
   // if (false) {
   //     cpe.fetch(0b0011011, 0b00, 0b00,0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   //     cpe.fetch(0b0001111, 0b00, 0b00, 0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   //     cpe.fetch(0b0001111, 0b00, 0b00, 0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   // }
   // // now with LI = 1
   // if (false) {
   //     cpe.RO = 0;
   //     // LMM : load M in AC            \/M
   //     cpe.fetch(0b0011011, 0b00, 0b00,0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   //     std::cout << "10 is loaded into AC" << std::endl;
   //     std::cout << "shift right, LI = 1" << std::endl;
   //     cpe.fetch(0b0001111, 0b00, 0b00,0b10,0,1);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   //     std::cout << "shift right, LI = 0" << std::endl;
   //     cpe.fetch(0b0001111, 0b00, 0b00, 0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   //     std::cout << "shift right, LI = 0" << std::endl;
   //     cpe.fetch(0b0001111, 0b00, 0b00, 0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   //     std::cout << "shift right, LI = 0" << std::endl;
   //     cpe.fetch(0b0001111, 0b00, 0b00, 0b10,0,0);
   //     cpe.decode();
   //     cpe.execute();
   //     std::cout << std::bitset<2>(cpe.MEM[AC]) << ", RO = " << (unsigned)cpe.RO << std::endl;
   // }
   return 0;
}
