#include "emulator.hpp"
#include<QDebug>
#include <iostream>

#define SETUP 1
#define SEND 2
#define RECIEVE 4

#define UART 1
#define I2C 2
#define SPI 4
#define GPIO1 8
#define GPIO2 16

MK589::MK589() {
    cpe_arr.resize(cpe_amount);
    this->reset();
}
MK589& MK589::operator=(const MK589 &mk) {
    // Guard self assignment
    if (this == &mk)
        return *this;
    this->reset();
    cpe_arr.resize(mk.cpe_amount);
    MAR = mk.MAR;
    CO = mk.CO;
    RO = mk.RO;
    CI = mk.CI;
    LI = mk.LI;
    D = mk.D;
    A = mk.A;
    for (size_t i = 0; i < 0xC; ++i) {
        MEM[i] = mk.MEM[i];
    }
    for (size_t i = 0; i < cpe_amount; ++i) {
        cpe_arr[i].reset();
    }
    rom.memory = mk.rom.memory;
    return *this;
}

MK589::MK589(const MK589& mk) {
    cpe_arr.resize(mk.cpe_amount);
    MAR = mk.MAR;
    CO = mk.CO;
    RO = mk.RO;
    CI = mk.CI;
    LI = mk.LI;
    D = mk.D;
    A = mk.A;
    for (size_t i = 0; i < 0xC; ++i) {
        MEM[i] = mk.MEM[i];
    }
    for (size_t i = 0; i < cpe_amount; ++i) {
        cpe_arr[i].reset();
    }
    rom.memory = mk.rom.memory;
}

void MK589::reset() {
    MAR = 0x0000;
    A.reset();
    D.reset();
    RO.reset();
    CO.reset();
    CI = 0b0;
    LI = 0b0;
    M = 0x0000;
    I = 0x0000;
    for (size_t i = 0; i < 0xC; ++i) {
        MEM[i] = 0;
    }
    for (size_t i = 0; i < cpe_amount; ++i) {
        cpe_arr[i].reset();
    }
    mcu.reset();
}


int MK589::send(const char package[4]) {
    DWORD dwSize = sizeof(package);   // размер этой строки
    DWORD dwBytesWritten;    // тут будет количество собственно переданных байт

    BOOL iRet = WriteFile(this->hSerial, package, 4, &dwBytesWritten, NULL);
    qInfo() << 4 << " Bytes in string. " << dwBytesWritten << " Bytes sended. " << "\n";
    for (size_t i = 0; i < dwSize; i++)
    {
        qInfo() << (int)package[i] << " ";
    }
    qInfo() << "\n";
    return iRet;
}

void MK589::ReadCOM(uint8_t num_reg)
{
    qInfo() << "Пришел в ридком1";
    COMMTIMEOUTS tTimeout;
    tTimeout.ReadIntervalTimeout = MAXWORD;
    tTimeout.ReadTotalTimeoutMultiplier = 0;
    tTimeout.ReadTotalTimeoutConstant = 2500    ; // pas de time out = 0
    tTimeout.WriteTotalTimeoutMultiplier = 0;
    tTimeout.WriteTotalTimeoutConstant = 2500;
    if (!SetCommTimeouts((HANDLE)hSerial, &tTimeout))
    {
        return;
    }
    DWORD iSize;
    char sReceivedChar[6];
    ReadFile(hSerial, &sReceivedChar[0], 6, &iSize, 0);  // получаем 1 байт
    if (iSize > 0) {   // если что-то принято, выводим.
        qInfo() << iSize;
        for(int i = 0; i < iSize; ++i){
            qInfo() << (int)sReceivedChar[i];
        }

        cpe_arr[0].MEM[num_reg] = int(sReceivedChar[4]);
        return;
    }
}

void MK589::ReadCOMGPIO(uint8_t num_reg)
{
    qInfo() << "Пришел в ридком1";
    COMMTIMEOUTS tTimeout;
    tTimeout.ReadIntervalTimeout = MAXWORD;
    tTimeout.ReadTotalTimeoutMultiplier = 0;
    tTimeout.ReadTotalTimeoutConstant = 2500    ; // pas de time out = 0
    tTimeout.WriteTotalTimeoutMultiplier = 0;
    tTimeout.WriteTotalTimeoutConstant = 2500;
    if (!SetCommTimeouts((HANDLE)hSerial, &tTimeout))
    {
        return;
    }
    DWORD iSize;
    char sReceivedChar[8];
    ReadFile(hSerial, &sReceivedChar[0], 8, &iSize, 0);  // получаем 1 байт
    if (iSize > 0) {   // если что-то принято, выводим.
        qInfo() << iSize;
        for(int i = 0; i < iSize; ++i){
            qInfo() << (int)sReceivedChar[i];
        }

        cpe_arr[0].MEM[num_reg] = int(sReceivedChar[6]);
        return;
    }
}



void MK589::do_fetch_decode_execute_cycle(const microcommand &mc) {
    mcu.X = std::bitset<8> ((mc.M & 0xFF00) >> 8);
    mcu.fetch(mc.AC, mcu.X, mc.FC, mc.LD);
    fetch_cpe(mc.F, mc.K, mc.I, mc.M, mc.ED, mc.EA);

    if(!(mc.f_ext == "00000000")){
        // type command | num_reg
        // 0001**** - setup, uart-write
        // 0010**** - send from **** reg to stm by uart
        // 0011**** - setup uart-read, read to **** reg from stm by uart
        // 0100**** - setup GPIOA
        // 0101**** - write GPIOA from **** reg
        // 0110**** - read GPIOA to **** reg
        // 0111**** - write GPIOE from **** reg
        // 1000**** - read GPIOE to **** reg
        // 1001**** - write GPIOE from **** reg
        // 1010**** - read GPIOE to **** reg


        qInfo() << "СТОЮ НА КОМАНДЕ ОТПРАВИТЬ\n";
        std::string r_group = mc.f_ext.substr(4, 4); // num_reg
        std::string f_group = mc.f_ext.substr(0, 4); // num_reg

        uint8_t num_reg =  std::stoi(r_group, 0, 2);
        if(f_group == "0001"){
            char data_setup[4] = { 66,0b00001111,(char)0b10000000,(char)194 };
            send(data_setup);
        }
        else if(f_group == "0010"){
            char data_write[4] = {66,0b00000011,0,(char)194};
            data_write[2] = (char)MEM[num_reg];
            qInfo() << "REg is "  << data_write[2];
            send(data_write);
        }
        else if(f_group == "0011"){
            char data_read[4] = { 34,0b00000011,(char)138,(char)162 };
            send(data_read);
            Sleep(100);
            ReadCOM(num_reg);
        }
        else if(f_group == "0100"){
            char data[4] = { 66,0b00001111,0b00000000,194 };
            send(data);
        }
        else if(f_group == "0101"){
            char data2[4] = { 66,0b00000001,0,194 };
            data2[2] = (char)MEM[num_reg];
            send(data2);
            qInfo() << "REg is "  << data2[2];
        }
        else if(f_group == "0110"){
            char data1[4] = { 34,0b00000001,138,162 };
            send(data1);
            Sleep(100);
            ReadCOMGPIO(num_reg);
        }
        else if(f_group == "0111"){
            char data2[4] = { 66,0b00001010,0,194 };
            data2[2] = (char)MEM[num_reg];
            send(data2);
            qInfo() << "REg is "  << data2[2];
        }
        else if(f_group == "1000"){
            char data1[4] =  { 34,0b00001010,138,162 };
            send(data1);
            Sleep(100);
            ReadCOMGPIO(num_reg);
        }
        else if(f_group == "1001"){
            char data2[4] = { 66,0b00000111,0,194 };
            data2[2] = (char)MEM[num_reg];
            send(data2);
            qInfo() << "REg is "  << data2[2];
        }
        else if(f_group == "1010"){
            char data1[4] =  { 34,0b00000111,138,162 };
            send(data1);
            Sleep(100);
            ReadCOM(num_reg);
        }
        else if(f_group == "1011"){
            char data[4] = { 66,0b000001111,0b01000000,194 };
            send(data);
        }
        else if(f_group == "1100"){
            char data1[4] = { 66,0b00100111,0,194 };
            data1[2] = (char)MEM[num_reg];
            send(data1);
        }
        else if(f_group == "1101"){
            char data1[4] = { 34,0b00100111,156,162 };
            send(data1);
            Sleep(100);
            ReadCOMGPIO(num_reg);
        }
        else if(f_group == "1110"){
            char data[4] = { 66,0b000001111,0b00100000,194 };
            send(data);
        }
        else if(f_group == "1111"){
            char data1[4] = { 66,0b10001010,0,194 };
            data1[2] = (char)MEM[num_reg];
            send(data1);
        }

        else{
            qInfo() << "Неизвестный тип микрокоманды!";
        }
    }

    decode(); // both mcu and cpe


    mcu.execute_output_flag_logic();
    this->FO = mcu.FO;

    A.reset();
    D.reset();
    CO = 0;
    RO = 0;
    if (is_performing_right_rot) {
        LI = FO;
        CO.reset();
        execute_cpe_right_rot();
    } else {
        CI = FO;
        execute_cpe();
    }

    if (ED == 0b1) {
       D = MEM[AC];
    }
    if (EA == 0b1) {
        A = MAR;
    }
    // when FI flag is set (after cpe execution)
    if (CO) {
        FI = *CO;
    } else {
        FI = *RO;
    }
    mcu.fetch_flag(FI);
    mcu.execute_input_flag_logic();
    mcu.execute_next_address_logic();
    decode_adr();
}
void MK589::decode_adr() {
    row_adr = (mcu.MA >> 4).to_ulong();
    std::string ma = mcu.MA.to_string();
    col_adr = (mcu.MA & std::bitset<9> {0b000001111}).to_ulong();
}

size_t MK589::get_row_adr() {
    return row_adr;
}

size_t MK589::get_col_adr() {
    return col_adr;
}

void MK589::load(std::bitset<8> x) {
    mcu.fetch(0, x, 0, 0b1);
    mcu.execute_next_address_logic();
    decode_adr();
}

void MK589::decode() {
    decode_cpe();
    mcu.decode();
}

void MK589::fetch_cpe(std::bitset<7> f,
                               WORD k,
                               WORD i,
                               WORD m,
                               BYTE ed,
                               BYTE ea)
{
    this->F = f;
    this->I = i;
    this->K = k;
    this->M = m;
    this->ED = ed;
    this->EA = ea;
}

void MK589::decode_cpe() {
    f_group = (F >> 4).to_ulong();
    r_group = (F & std::bitset<7> {0b0001111}).to_ulong();

    if(F == 0b1111111){
        f_group = 8;
        r_group = 4;
    }
    else if (r_group == 0xA or r_group == 0xB) {
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
    for (size_t i = 0; i < cpe_amount; ++i) {
        cpe_arr[i].r_group = r_group;
        cpe_arr[i].f_group = f_group;
        cpe_arr[i].ADR = ADR;
    }
    if (r_group == 3 and f_group == 0) {
        is_performing_right_rot = true;
    } else {
        is_performing_right_rot = false;
    }
}

void MK589::execute_cpe_right_rot() {
    for (size_t i = cpe_amount - 1; i < cpe_amount; --i) {
        cpe_arr[i].fetch(F, ((I >> (i*2)) & 0b11), ((K >> (i*2)) & 0b11),
                ((M >> (i*2)) & 0b11), 0, LI, ED, EA);
        cpe_arr[i].execute();
        LI = *cpe_arr[i].RO;
    }
    unite_registers();
    RO = *cpe_arr[0].RO;
    CO.reset();
    //FI = RO.value();
}

void MK589::execute_cpe() {
    for (size_t i = 0; i < cpe_amount; ++i) {
        cpe_arr[i].fetch(F, ((I >> (i*2)) & 0b11), ((K >> (i*2)) & 0b11),
                ((M >> (i*2)) & 0b11), CI, 0, ED, EA);
        cpe_arr[i].execute();
        CI = *cpe_arr[i].CO;
        if(F == 0b1111111){
            break;
        }
    }
    unite_registers();
    CO = *cpe_arr[cpe_amount - 1].CO;
    RO.reset();
   // FI = CO.value();
}

void MK589::unite_registers() {
    for (size_t i = 0; i < 0xC; ++i) {
        MEM[i] = (cpe_arr[7].MEM[i] << 14) |
                (cpe_arr[6].MEM[i] << 12) |
                (cpe_arr[5].MEM[i] << 10) |
                (cpe_arr[4].MEM[i] << 8) |
                (cpe_arr[3].MEM[i] << 6) |
                (cpe_arr[2].MEM[i] << 4) |
                (cpe_arr[1].MEM[i] << 2) |
                (cpe_arr[0].MEM[i] << 0);
    }
    MAR = (cpe_arr[7].MAR << 14) |
            (cpe_arr[6].MAR << 12) |
            (cpe_arr[5].MAR << 10) |
            (cpe_arr[4].MAR << 8) |
            (cpe_arr[3].MAR << 6) |
            (cpe_arr[2].MAR << 4) |
            (cpe_arr[1].MAR << 2) |
            (cpe_arr[0].MAR << 0);
}
