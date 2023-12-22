#include "cpe.h"
#include<QDebug>

#include <iostream>
using namespace std;


CPE::CPE() {    
    this->reset();
}

void CPE::reset() {
    for (size_t i = 0x0; i < 0xE; ++i) {
        CPE::MEM[i] = 0b00;
    }
    MAR = 0b00;
    A.reset();
    D.reset();
    RO.reset();
    CO.reset();
    X = 0b0;
    Y = 0b0;
}

BYTE CPE::word_wise_or(BYTE op) {
    return get_lb(op) | get_hb(op);
}

BYTE CPE::get_lb(BYTE src) {
    BUF1 = src;
    return BUF1;
}
BYTE CPE::get_hb(BYTE src) {
    BUF1 = (src >> 1);
    return BUF1;
}

void CPE::fetch(const std::bitset<7>& f, BYTE i, BYTE k, BYTE m, BYTE CI,
                BYTE LI, BYTE ed, BYTE ea) {
    this->F = f;
    this->I = i;
    this->K = k;
    this->CI = CI;
    this->LI = LI;
    this->M = m;
    this->ED = ed;
    this->EA = ea;
}

void CPE::decode() {
    f_group = (F >> 4).to_ulong();
    r_group = (F & std::bitset<7> {0b0001111}).to_ulong();

    if (r_group == 0xA or r_group == 0xB) {
        ADR = r_group;
        r_group = 2;
    } else if (r_group == 0xE or r_group == 0xF) {
        ADR = r_group - 4;
        r_group = 3;
    } else {
        if (r_group == 0xC or r_group == 0xD) {
            ADR = r_group - 2;
        } else {
            ADR = r_group;
        }
        r_group = 1;
    }
}
void CPE::execute() {
    A.reset();
    D.reset();
    RO = 0;
    CO = 0;

    switch(f_group) {
        case 0:
            execute_f0();
            break;
        case 1:
            execute_f1();
            break;
        case 2:
            execute_f2();
            break;
        case 3:
            execute_f3();
            break;
        case 4:
            execute_f4();
            break;
        case 5:
            execute_f5();
            break;
        case 6:
            execute_f6();
            break;
        case 7:
            execute_f7();
            break;
        case 8:
            execute_f8();
            break;
    }
    if (CO) {
        RO.reset();
    }
    if (RO) {
        CO.reset();
    }
    // set buses output
    if (ED == 0b1) {
       D = MEM[AC];
    }
    if (EA == 0b1) {
        A = 0;
        A.value() = MAR;
    }
}
void CPE::propogate() {
    if (f_group > 3) {
        Y = 1;
        X = CO.value();
    } else {
        BYTE a1 = (opA >> 1) & 0b1;
        BYTE a0 = opA & 0b1;
        BYTE b0 = opB & 0b1;
        BYTE b1 = (opB >> 1) & 0b1;
        X = (a1 & b1) | (a0 & b0);
        Y = (a1 & b1) | ((a1 | b1) & (a0 | b0));
    }
}
void CPE::compute_CO() {
    *CO = (CI & Y) | (X & Y);
}
void CPE::execute_f0() {
    switch(r_group) {
        case 1:
            opA = MEM[ADR];
            opB = (MEM[AC] & K);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            MEM[AC] = BUF2;
            break;
        case 2:
            opA = M;
            opB = (MEM[AC] & K);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 3:
            CO.reset();
            RO = get_lb(MEM[ADR]) & ~(get_lb(I) & get_lb(K));
            // low bit of AT
            BUF1 = (get_lb(MEM[ADR]) & (get_lb(I) & get_lb(K)))
                 | (get_hb(MEM[ADR]) | (get_hb(I) & get_hb(K)));
            if (BUF1 == 0b0) {
                MEM[ADR] &= 0b10;
            } else {
                MEM[ADR] |= 0b01;
            }
            // high bit of AT
            BUF1 = LI | ((get_hb(I) & get_hb(K)) & get_hb(MEM[ADR]));
            if (BUF1 == 0b0) {
                MEM[ADR] &= 0b01;
            } else {
                MEM[ADR] |= 0b10;
            }
            break;
    }
}
void CPE::execute_f1() {
    switch(r_group) {
        case 1:
            MAR = K | MEM[ADR];
            opA = MEM[ADR];
            opB = K;
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 2:
            MAR = K | M;
            opA = M;
            opB = K;
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 3:
            opA = (~MEM[ADR] | K);
            opB = (MEM[ADR] & K);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
    }
}
void CPE::execute_f2() {
    switch(r_group) {
        case 1:
            opA = (MEM[AC] & K);
            opB = (-1);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 2:
            opA = (MEM[AC] & K);
            opB = (-1);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 3:
            opA = (I & K);
            opB = (-1);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
    }
}
void CPE::execute_f3() {
    switch(r_group) {
        case 1:
            opA = MEM[ADR];
            opB = (MEM[AC] & K);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 2:
            opA = M;
            opB = (MEM[AC] & K);
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
        case 3:
            opA = MEM[ADR];
            opB = I & K;
            propogate();
            compute_CO();
            BUF2 = opA + opB + CI;
            MEM[ADR] = BUF2;
            break;
    }
}
void CPE::execute_f4() {
    switch(r_group) {
        case 1:
            BUF1 = CI | word_wise_or(MEM[ADR] & MEM[AC] & K);
            BUF2 = MEM[ADR] & (MEM[AC] & K);
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 2:
            BUF1 = CI | word_wise_or(M & MEM[AC] & K);
            BUF2 = M & (MEM[AC] & K);
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 3:
            BUF1 = CI | word_wise_or(I & MEM[ADR] & K);
            BUF2 = MEM[ADR] & (I & K);
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
    }
}
void CPE::execute_f5() {
    switch(r_group) {
        case 1:
            BUF1 = word_wise_or(MEM[ADR] & K) | CI;
            BUF2 = MEM[ADR] & K;
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 2:
            BUF1 = word_wise_or(M & K) | CI;
            BUF2 = M & K;
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 3:
            BUF1 = word_wise_or(MEM[ADR] & K) | CI;
            BUF2 = MEM[ADR] & K;
            MEM[ADR] = BUF2;
            *CO = BUF1;
            propogate();
            break;
    }
}
void CPE::execute_f6() {
    switch(r_group) {
        case 1:
            BUF1 = word_wise_or(MEM[AC] & K) | CI;
            BUF2 = (MEM[AC] & K) | MEM[ADR];
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 2:
            BUF1 = word_wise_or(MEM[AC] & K) | CI;
            BUF2 = (MEM[AC] & K) | M;
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 3:
            BUF1 = word_wise_or(I & K) | CI;
            BUF2 = (I & K) | MEM[ADR];
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
    }
}
void CPE::execute_f7() {
    switch(r_group) {
        case 1:
            BUF1 = word_wise_or(MEM[AC] & MEM[ADR] & K) | CI;
            BUF2 = ~((MEM[AC] & K) ^ MEM[ADR]);
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 2:
            BUF1 = word_wise_or(MEM[AC] & M & K) | CI;
            BUF2 = ~((MEM[AC] & K) ^ M);
            MEM[ADR] = BUF2;
            CO.value() = BUF1;
            propogate();
            break;
        case 3:
            BUF1 = word_wise_or(MEM[ADR] & I & K) | CI;
            BUF2 = ~((I & K) ^ MEM[ADR]);
            MEM[ADR] = BUF2;
            *CO = BUF1;
            propogate();
            break;
    }
}

/* заглушка для общения с внешним устр-вом */
void CPE::execute_f8() {
//    char data[] = "123456789";  // строка для передачи
//    DWORD dwSize = sizeof(data);   // размер этой строки
//    DWORD dwBytesWritten;    // тут будет количество собственно переданных байт

//    BOOL iRet = WriteFile(hSerial, data, dwSize, &dwBytesWritten, NULL);

//    qInfo() << dwSize << " Bytes in string. " << dwBytesWritten << " Bytes sended. " << "\n";

//    if(dwBytesWritten == 10){
//        qInfo() << "Общение с внешним устройством успешно!" << "\n";
//    }
//    else{
//        qInfo() << "ОШИБКА!" << "\n";
//    }
}
