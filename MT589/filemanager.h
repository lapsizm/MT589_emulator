#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include <fstream>
#include <emulator.hpp>
#include <string>
#include <json.hpp>

using nlohmann::json;

namespace fm {

    // RAM

    std::vector<std::string> get_ram(const std::string& filename);

    void save_ram(const std::string& filename, const std::vector<std::string>& data);

    // ISA

    struct isa_data {
        std::map<std::string, std::string> isa_regs;
        std::map<std::string, WORD> isa_commands;
    };

    void save_isa(const std::string& filename, const isa_data& data);

    isa_data get_isa_data(const std::string& filename);

    // ROM

    struct programm_data {
        MK589 mk;
        int start_row;
        int start_col;
    };

    programm_data get_data(const std::string& filename);

    void save(const std::string& filename, MK589& mk, int startCol, int startRow, MT::Mode mode);

    // JSON Save & Read

    void write_to_file(const std::string& filename, const json& data);

    std::string read_from_file(const std::string& filename);

}

#endif // FILEMANAGER_H
