#ifndef RAM_H
#define RAM_H

#include "mt_global.h"

struct RAM {
    RAM();
    void write(size_t address, WORD data);
    WORD read(size_t address);

    size_t size = 0x100;
    std::vector<WORD> memory;
};

#endif // RAM_H
