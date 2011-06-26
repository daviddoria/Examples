#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QMainWindow>

class Form : public QMainWindow, private Ui::MainWindow
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    

};

#endif
