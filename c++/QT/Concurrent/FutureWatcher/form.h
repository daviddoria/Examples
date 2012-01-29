#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QFutureWatcher>

#include "MyClass.h"

class Form : public QWidget, private Ui::Form
{
Q_OBJECT

public slots:

  void slot_finished();
  void on_pushButton_clicked();

public:
    Form(QWidget *parent = 0);

private:
  QFutureWatcher<void> FutureWatcher;
  MyClass MyObject;
};

#endif
