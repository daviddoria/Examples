#ifndef CALCULATORFORM_H
#define CALCULATORFORM_H

#include <QMainWindow>

#include "ui_main.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QMainWindow *parent = 0);

public slots:

    void comboBox_Activated();
    
private:
    Ui::MainWindow ui;
};

#endif
