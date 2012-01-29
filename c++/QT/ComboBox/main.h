#ifndef CALCULATORFORM_H
#define CALCULATORFORM_H

#include <QMainWindow>

#include "ui_main.h"

class MainWindow : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MainWindow(QMainWindow *parent = 0);

public slots:

    void on_comboBox_activated(int);

};

#endif
