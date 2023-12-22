#ifndef CREATEISA_H
#define CREATEISA_H

#include <QDialog>
#include <QTableWidgetItem>
#include <QLabel>

namespace Ui {
class CreateISA;
}

class CreateISA : public QDialog
{
    Q_OBJECT

public:
    explicit CreateISA(QWidget *parent = nullptr);
    ~CreateISA();

    CreateISA& operator=(const CreateISA &rhs) {
        // Guard self assignment
        if (this == &rhs)
            return *this;
        this->regsNNames = rhs.regsNNames;
        this->commandNames = rhs.commandNames;
        this->commandAddresses = rhs.commandAddresses;
        return *this;
    }

    void prepare();

private slots:
    void on_saveButton_clicked();

private:
    Ui::CreateISA *ui;

   std::vector<QTableWidgetItem*> regsNNames;

   std::vector<QTableWidgetItem*> commandNames;

   std::vector<QTableWidgetItem*> commandAddresses;

   std::vector<std::string> default_regs = {
       "REG0",
       "REG1",
       "REG2",
       "REG3",
       "REG4",
       "REG5",
       "REG6",
       "REG7",
       "REG8",
       "REG9"
   };
};

#endif // CREATEISA_H
