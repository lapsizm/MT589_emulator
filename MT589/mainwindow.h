#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLCDNumber>
#include <emulator.hpp>
#include <QListWidgetItem>
#include <QPushButton>
#include <QTableWidgetItem>
#include <QAbstractItemView>
#include <Model.h>
#include <memory>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    HANDLE hSerial;
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    HANDLE getSerial(){
        return hSerial;
    }
    static void Hell(){
        qInfo() << "jhell";
    }
    void ShowCOMPorts();

    void add_cpe_funcs();

    void add_fc1_funcs();

    void add_fc2_funcs();

    void add_jump_funcs();

    // Help functions

    void setupRegs();

    void setLCDsColor();

    void setupBoxes();

    void clearInputs();

    void update_on_cpu_data();

    void handleInputState();

    void fillInputs();

    void setItemColor(Point point);

    void configUIMode();

    void setMode(Mode mode);

    void changeCurrentPoint(Point last, Point currentPoint);

    void setupMatrix();

    void setupRAM();

    void setupItems();

    MK589 mk;

private slots:
    void on_stepButton_clicked();

    void on_runButton_clicked();

    void on_boxCPE_currentIndexChanged(int index);

    void on_boxJUMP_currentIndexChanged(int index);

    void on_commandAddressEdit_textEdited(const QString &arg1);

    void on_iLineEdit_textEdited(const QString &arg1);

    void on_kLineEdit_textEdited(const QString &arg1);

    void on_mLineEdit_textEdited(const QString &arg1);

    void on_riLineEdit_textEdited(const QString &arg1);

    void on_ciLineEdit_textEdited(const QString &arg1);

    void on_Ext_code_textEdited(const QString &arg1);

    void on_tableWidget_currentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn);

    void on_saveButton_clicked();

    void on_clearButton_clicked();

    void on_startAddressEdit_textEdited(const QString &arg1);

    void on_loadButton_clicked();

    void on_ramcLineEdit_textEdited(const QString &arg1);

    void on_endButton_clicked();

    void on_tableWidget_cellClicked(int row, int column);

    void on_fLineEdit_textEdited(const QString &arg1);

    void on_ramWidget_cellChanged(int row, int column);

    void on_resetButton_clicked();

    void on_save_file_as_triggered();

    void on_open_file_triggered();

    void on_save_file_triggered();

    void on_open_command_mode_triggered();

    void on_action_COM_triggered();

    void on_tableWidget_cellChanged(int row, int column);

private:

    // UI Objects

    Ui::MainWindow *ui;

    std::vector<QLCDNumber*> regLCDs = {};

    std::vector<std::vector<QTableWidgetItem*>> matrixItems;

    std::vector<QTableWidgetItem*> ramItems;

    // States & Data

    Model model = Model();

    QBrush commandColor = QBrush(Qt::blue);

    QBrush startColor = QBrush(Qt::green);

    QBrush currentRunningColor = QBrush(Qt::red);

    QBrush transparentColor = QBrush(Qt::transparent);

    bool loaded = false;
};

#endif // MAINWINDOW_H
