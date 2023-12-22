#include "mcu.h"

void MCU::reset() {
    FC_10 = 0b00;
    FC_32 = 0b00;
    TF = 0b0;
    TC = 0b0;
    TZ = 0b0;
    PR_latch = 0x0;
    MPAR = 0b000000000;
    FI = 0b0;
    FO = 0b0;
    LD = 0b0;
    MA = 0b000000000;
}

MCU::MCU() {
    this->reset();
}

void MCU::fetch(std::bitset<7> ac, std::bitset<8> x, BYTE fc, BYTE ld) {
    this->AC = ac;
    this->X = x;
    this->LD = ld;
    this->FC_10 = fc & 0b0011;
    this->FC_32 = (fc >> 2) & 0b11;

    for (size_t i = 0; i < 3; ++i) {
        PR_latch[i] = X[i];
    }
}

void MCU::fetch_flag(BYTE fi) {
    this->FI = fi;
}

void MCU::decode_jmp() {
    if (AC.to_string().substr(0, 2) == "00") {
        cur_jmp = JUMP::JCC;
    } else if (AC.to_string().substr(0, 3) == "010") {
        cur_jmp = JUMP::JZR;
    } else if (AC.to_string().substr(0, 3) == "011") {
        cur_jmp = JUMP::JCR;
    } else if (AC.to_string().substr(0, 3) == "111") {
        cur_jmp = JUMP::JCE;
    } else if (AC.to_string().substr(0, 3) == "100") {
        cur_jmp = JUMP::JFL;
    } else if (AC.to_string().substr(0, 4) == "1010") {
        cur_jmp = JUMP::JCF;
    } else if (AC.to_string().substr(0, 4) == "1011") {
        cur_jmp = JUMP::JZF;
    } else if (AC.to_string().substr(0, 4) == "1100") {
        cur_jmp = JUMP::JPR;
    } else if (AC.to_string().substr(0, 4) == "1101") {
        cur_jmp = JUMP::JLL;
    } else if (AC.to_string().substr(0, 5) == "11111") {
        cur_jmp = JUMP::JRL;
    } else if (AC.to_string().substr(0, 5) == "11110") {
        cur_jmp = JUMP::JPX;
    }
}
void MCU::decode_fl() {
    cur_FCI = static_cast<FCI>(FC_10);
    cur_FCO = static_cast<FCO>(FC_32);
    switch (FC_32) {
        case 0: {
            cur_FCO = FCO::FF0;
            break;
        }
        case 1: {
            cur_FCO = FCO::FFC;
            break;
        }
        case 2: {
            cur_FCO = FCO::FFZ;
            break;
        }
        case 3: {
            cur_FCO = FCO::FF1;
            break;
        }
    }
}

void MCU::execute_input_flag_logic() {
    TF = FI;
    switch(cur_FCI) {
        case FCI::SCZ:
            TC = TF;
            TZ = TF;
            break;
        case FCI::STZ:
            TZ = TF;
            break;
        case FCI::STC:
            TC = TF;
            break;
        case FCI::HCZ:
            break;
    }
}
void MCU::execute_output_flag_logic() {
    switch(cur_FCO) {
        case FCO::FF0:
            FO = 0b0;
            break;
        case FCO::FFC:
            FO = TC;
            break;
        case FCO::FFZ:
            FO = TZ;
            break;
        case FCO::FF1:
            FO = 0b1;
            break;
    }
}


void MCU::execute_next_address_logic() {
    if (LD == 0b1) {
        MPAR[8] = 0;
        for (size_t i = 0; i < 4; ++i) {
            MPAR[i] = X[i + 4];
        }
        for (size_t i = 4; i < 8; ++i) {
            MPAR[i] = X[i - 4];
        }
    } else {
        switch(cur_jmp) {
            case JUMP::JCC:
                for (size_t i = 4; i < 9; ++i) {
                    MPAR[i] = AC[i - 4];
                }

                break;
            case JUMP::JZR:
                for (size_t i = 0; i < 9; ++i) {
                    MPAR[i] = 0;
                }
                for (size_t i = 0; i < 4; ++i) {
                    MPAR[i] = AC[i];
                }
                break;
            case JUMP::JCR:
                for (size_t i = 0; i < 4; ++i) {
                    MPAR[i] = AC[i];
                }

                break;
            case JUMP::JCE:
                for (size_t i = 4; i < 7; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                // PR latch output enable?
                break;
            case JUMP::JFL:
                for (size_t i = 4; i < 8; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                MPAR[2] = 0;
                MPAR[1] = 1;
                MPAR[0] = TF;
                break;
            case JUMP::JCF:
                for (size_t i = 4; i < 7; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                MPAR[2] = 0;
                MPAR[1] = 1;
                MPAR[0] = TC;
                break;
            case JUMP::JZF:
                for (size_t i = 4; i < 7; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                MPAR[2] = 0;
                MPAR[1] = 1;
                MPAR[0] = TZ;
                break;
            case JUMP::JPR:
                for (size_t i = 4; i < 7; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                for (size_t i = 0; i < 4; ++i) {
                    MPAR[i] = PR_latch[i];
                }
                break;
            case JUMP::JLL:
                for (size_t i = 4; i < 7; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                MPAR[3] = 0;
                MPAR[2] = 1;
                MPAR[1] = PR_latch[3];
                MPAR[0] = PR_latch[2];
                break;
            case JUMP::JRL:
                for (size_t i = 4; i < 7; ++i) {
                    MPAR[i] = AC[i - 4];
                }
                MPAR[3] = 1;
                MPAR[2] = 1;
                MPAR[1] = PR_latch[1];
                MPAR[0] = PR_latch[0];
                break;
            case JUMP::JPX:
                MPAR[5] = AC[1];
                MPAR[4] = AC[0];
                for (size_t i = 0; i < 4; ++i) {
                    MPAR[i] = X[i + 4];
                }
                break;
        }
    }
    MA = MPAR;
}
void MCU::decode() {
    decode_jmp();
    decode_fl();
}

