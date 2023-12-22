#ifndef ROMWINDOW_H
#define ROMWINDOW_H

#include <QDialog>
#include <emulator.hpp>
#include <QTableWidgetItem>

namespace Ui {
class ROMWindow;
}

class ROMWindow : public QDialog
{
    Q_OBJECT

public:
    explicit ROMWindow(QWidget *parent = nullptr);
    ~ROMWindow();

    void setupItems();


    MK589 mk;

private:
    Ui::ROMWindow *ui;

    QBrush startColor = QBrush(Qt::green);

    QBrush currentRunningColor = QBrush(Qt::red);

    QBrush transparentColor = QBrush(Qt::transparent);

    std::vector<std::vector<QTableWidgetItem*>> items;
};

#endif // ROMWINDOW_H
