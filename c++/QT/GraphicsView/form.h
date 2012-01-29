#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class QGraphicsPixmapItem;

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);
protected:
  void showEvent ( QShowEvent * event );
  void resizeEvent ( QResizeEvent * event );
  
  QGraphicsPixmapItem* BlackItem;
};

#endif
