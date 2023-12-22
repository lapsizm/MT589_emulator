#include "createisa.h"
#include "ui_createisa.h"
#include "filemanager.h"
#include <QFileDialog>

CreateISA::CreateISA(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CreateISA)
{
    ui->setupUi(this);
    prepare();

}

void CreateISA::prepare() {
    QStringList verlist;
    for (size_t i = 0; i < 10; ++i) {
        verlist << default_regs[i].c_str();
    }

    ui->regsTableWidget->setVerticalHeaderLabels(verlist);
    ui->regsTableWidget->setHorizontalHeaderLabels({"New name"});

   for (size_t i = 0; i < 10; ++i) {
       QTableWidgetItem* item = new QTableWidgetItem;
       ui->regsTableWidget->setItem(i, 0, item);
       regsNNames.push_back(item);
   }

   ui->commandsTableWidget->setHorizontalHeaderLabels({"Command", "Address"});
   for (size_t i = 0; i < 31; ++i) {
       QTableWidgetItem* item = new QTableWidgetItem;
       ui->commandsTableWidget->setItem(i, 0, item);
       commandNames.push_back(item);
   }
   for (size_t i = 0; i < 31; ++i) {
       QTableWidgetItem* item = new QTableWidgetItem;
       item->setData(Qt::ItemDataRole::EditRole, 0);
       ui->commandsTableWidget->setItem(i, 1, item);
       commandAddresses.push_back(item);
   }
}

CreateISA::~CreateISA()
{
    delete ui;
}

void CreateISA::on_saveButton_clicked()
{
    fm::isa_data data;
    for (size_t i = 0; i < regsNNames.size(); ++i) {
        std::string regNName = regsNNames[i]->text().toStdString();
        if (!regNName.empty()) {
            data.isa_regs[default_regs[i]] = regNName;
        }
    }

    for (size_t i = 0; i < commandNames.size(); ++i) {
        std::string command = commandNames[i]->text().toStdString();
        size_t address = commandAddresses[i]->text().toUInt();
        if (!command.empty()) {
            data.isa_commands[command] = address;
        }
    }

    std::string filename = QFileDialog::getSaveFileName(this, tr("Save ISA"),
                               "~/prog.isad",
                               tr("*.isad")).toStdString();
    if (filename.empty()) { return; }
    fm::save_isa(filename, data);
}

