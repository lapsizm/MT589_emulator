#include "romwindow.h"
#include "ui_romwindow.h"

ROMWindow::ROMWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ROMWindow)
{
    ui->setupUi(this);
    QStringList horlist;
    for (size_t i = 0; i < 16; ++i) {
        horlist << std::to_string(i).c_str();
    }
    QStringList verlist;
    for (size_t i = 0; i < 32; ++i) {
        verlist << std::to_string(i).c_str();
    }

    ui->tableWidget->setHorizontalHeaderLabels(horlist);
    ui->tableWidget->setVerticalHeaderLabels(verlist);
    ui->tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);

    for (size_t i = 0; i < 32; ++i) {
        std::vector<QTableWidgetItem*> row;
        for (size_t j = 0; j < 16; ++j) {
            QTableWidgetItem* item = new QTableWidgetItem();
            row.push_back(item);
            ui->tableWidget->setItem(i, j, item);
        }
        items.push_back(row);
    }
}

ROMWindow::~ROMWindow()
{
    delete ui;
}

void ROMWindow::setupItems() {
    for (size_t row = 0; row < 32; ++row) {
        for (size_t col = 0; col < 16; ++col) {
            microcommand command = mk.rom.read(row, col);
            QTableWidgetItem* item = items[row][col];
            if (command.empty) {
                item->setBackground(transparentColor);
            } else {
                item->setBackground(QBrush(Qt::blue));
            }
            item->setText(command.tag.c_str());
        }
    }
}
