#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class MyForm : public QWidget, private Ui::Form
{
Q_OBJECT
public:
    MyForm(QWidget *parent = 0);

public slots:
    void on_btnButton_clicked(); // By naming the slot on_[object]_[action]() you do not have to manually call connect()

protected:
  // Q_SLOT void on_btnButton_clicked(); // If you add Q_SLOT in front, you can put the declaration anywhere, not just in public slots:

};

#endif
