#ifndef MT_GLOBAL_H
#define MT_GLOBAL_H

#include <array>
#include <bitset>
#include <vector>
#include <optional>

using BYTE = uint8_t;
using WORD = uint16_t;

constexpr BYTE AC = 0xB;
constexpr BYTE T = 0xA;

namespace MT {
    enum Mode {
        microcommand,
        command
    };
}

#endif // MT_GLOBAL_H
