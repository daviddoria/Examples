#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget, private Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    
    void on_lineEdit1_textEdited ( const QString & text );
    void on_lineEdit2_textEdited ( const QString & text );
};

#endif
