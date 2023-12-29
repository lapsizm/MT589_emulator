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

int MK589::setup(int Type_out, int interface_type) {

    uint8_t data[7] = { 255,0,0 ,255,255,255,255 };  // строка для передачи
    data[1] = SETUP;
    data[2] = Type_out;
    data[3] = interface_type;
    DWORD dwSize = sizeof(data);   // размер этой строки
    DWORD dwBytesWritten;    // тут будет количество собственно переданных байт

    BOOL iRet = WriteFile(this->hSerial, data, dwSize, &dwBytesWritten, NULL);
    qInfo() << dwSize << " Bytes in string. " << dwBytesWritten << " Bytes sended. " << "\n";
    for (size_t i = 0; i < dwSize; i++)
    {
        qInfo() << (int)data[i] << " ";
    }
    qInfo() << "\n";
    return iRet;
}

int MK589::send(int interface_type, const char package[3]) {

    uint8_t data[7] = { 255,0,0 ,package[0], package[1] ,package[2] , 255 };  // строка для передачи
    data[1] = SEND;
    data[2] = interface_type;
    DWORD dwSize = sizeof(data);   // размер этой строки
    DWORD dwBytesWritten;    // тут будет количество собственно переданных байт

    BOOL iRet = WriteFile(this->hSerial, data, dwSize, &dwBytesWritten, NULL);
    qInfo() << dwSize << " Bytes in string. " << dwBytesWritten << " Bytes sended. " << "\n";
    for (size_t i = 0; i < dwSize; i++)
    {
        qInfo() << (int)data[i] << " ";
    }
    qInfo() << "\n";
    return iRet;
}

void MK589::ReadCOM(uint8_t num_reg)
{
    DWORD iSize;
    char sReceivedChar[1] = { 'H' };
    qInfo() << "HELLO" << num_reg;
    for(int i = 0; i < 3000000; ++i){
        //qInfo() << i << "\n";
        ReadFile(this->hSerial, sReceivedChar, 1, &iSize, NULL);  // получаем 1 байт
        if (iSize > 0){   // если что-то принято, выводим
            qInfo() << "Recieved = " << (int)sReceivedChar[0] << "\n";
            for(int i = 0; i < 1; ++i){
                cpe_arr[i].MEM[num_reg] = uint8_t(sReceivedChar[0]);
                qInfo() << "REG: " << cpe_arr[i].MEM[num_reg] << "\n";
            }
            break;
        }
    }

}


void MK589::do_fetch_decode_execute_cycle(const microcommand &mc) {
    mcu.X = std::bitset<8> ((mc.M & 0xFF00) >> 8);
    mcu.fetch(mc.AC, mcu.X, mc.FC, mc.LD);
    fetch_cpe(mc.F, mc.K, mc.I, mc.M, mc.ED, mc.EA);

    if(!(mc.f_ext == "00000000")){
        // type comand | type interface | num_reg
        // 0101**** - setup, uart, send,
        // 0110**** - setup, gpio1, send
        // 0111**** - setup, uart, recv
        // 1001**** - send, uart, from num **** of reg
        // 1010**** - send, gpio1, from num **** of reg
        // 1101**** - recv, uart, to num **** of reg
        // 1110**** - recv, gpio1, to num **** of reg

        qInfo() << "СТОЮ НА КОМАНДЕ ОТПРАВИТЬ\n";
        std::string r_group = mc.f_ext.substr(4, 4); // num_reg
        std::string f_group_0 = mc.f_ext.substr(0,2); // type comand
        std::string f_group_1 = mc.f_ext.substr(2,2); // type interface
        uint8_t num_reg =  std::stoi(r_group, 0, 2);
        char package[3] = {0, 0, 0};
        if(f_group_0 == "01"){
            // setup
            if(f_group_1 == "01"){
                //uart
                if (!setup(SEND, UART)) {
                    qInfo() << "FUck\n";
                }
            }
            else if(f_group_1 == "10"){
                //GPIO1
                if (!setup(SEND, GPIO1)) {
                    qInfo() << "FUck\n";
                }
            }
            else if(f_group_1 == "11"){
                //recv
                if (!setup(RECIEVE, UART)) {
                    qInfo() << "FUck\n";
                }
            }
            else{
                qInfo() << "Неизвестный тип микрокоманды!";
            }
        }
        else if(f_group_0 == "10"){
            // write
            if(f_group_1 == "01"){
                //uart
                package[2] = (char)MEM[num_reg];
                if (!send(UART, package)) {
                    qInfo() << "FUck\n";
                }

            }
            else if(f_group_1 == "10"){
                //GPIO1
                 package[2] = (char)MEM[num_reg];
                if (!send(GPIO1, package)) {
                    qInfo() << "FUck\n";
                }
            }
            else{
                qInfo() << "Неизвестный тип микрокоманды!";
            }
        }
        else if(f_group_0 == "11"){

            if(f_group_1 == "01"){
                //uart
                ReadCOM(num_reg);

            }
            else if(f_group_1 == "10"){
                //GPIO1
                package[2] = (char)MEM[num_reg];
                if (!send(GPIO1, package)) {
                    qInfo() << "FUck\n";
                }
            }
            else{
                qInfo() << "Неизвестный тип микрокоманды!";
            }
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
