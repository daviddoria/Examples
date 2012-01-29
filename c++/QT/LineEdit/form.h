#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    void btnUpdate_SetLabelText();
    void on_lineEdit_textEdited( const QString & text );

};

#endif
