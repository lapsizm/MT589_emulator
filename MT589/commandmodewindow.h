#ifndef COMMANDMODEWINDOW_H
#define COMMANDMODEWINDOW_H

#include <QLCDNumber>
#include <QMainWindow>
#include <romwindow.h>
#include <QTableWidgetItem>
#include <emulator.hpp>
#include <sstream>
#include <mainwindow.h>
#include <createisa.h>

namespace Ui {
class CommandModeWindow;
}

class CommandModeWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CommandModeWindow(QWidget *parent = nullptr);
    ~CommandModeWindow();

    MK589 mk;

    std::shared_ptr<MainWindow> mainWindow = nullptr;

    void prepareISAWindowText();

private slots:
    void on_open_rom_triggered();

    void on_save_triggered();

    void on_save_as_triggered();

    void on_open_triggered();

    void on_open_microcommand_mode_triggered();

    void on_resetButton_clicked();

    void on_stepButton_clicked();

    void on_runButton_clicked();

    void on_endButton_clicked();

    void on_ramWidget_cellChanged(int row, int column);

    void update_on_cpu_data();

    void setupRegs();

    void changeCurrentRow(WORD oldRow, WORD newRow);

    void on_load_isa_triggered();

    void on_load_rom_triggered();

    void on_createISAButton_clicked();

    void on_load_ram_triggered();

private:
    Ui::CommandModeWindow *ui;

    ROMWindow romWindow;

    CreateISA isaWindow;

    std::vector<std::shared_ptr<QLCDNumber>> regLCDs = {};

    std::vector<std::shared_ptr<QTableWidgetItem>> items;

    std::string toHex(unsigned int value);

    unsigned int parseHex(const std::string& str);

    WORD parseCommand(const std::string& str);

    std::map<std::string, WORD> isa_commands = {
        {"INCA", 0x0100},
        {"MOVA", 0x03},
        {"ADDA", 0x02},
        {"POP", 0x0500},
        {"PUSH", 0x04},
        {"IS", 0x0600},
        {"INVA", 0x0700},
        {"SHAR", 0x08},
        {"SHAL", 0x09},
        {"JMP", 0xA}
    };

    std::map<std::string, std::string> isa_regs = {
        {"REG0", "PC"},
        {"REG1", "SP"},
        {"REG8", "A"},
    };

    WORD* PC;

    bool loaded = false;

    bool mkwrite = false;

    std::vector<QLabel*> reg_labels;

    std::string current_filename = "";
};

#endif // COMMANDMODEWINDOW_H
