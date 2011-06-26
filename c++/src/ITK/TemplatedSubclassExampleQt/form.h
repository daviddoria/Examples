#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "Segmentation.h"

class MyForm : public QWidget, private Ui::Form
{
	Q_OBJECT
public:
  MyForm(QWidget *parent = 0);

  //Segmentation<T>* MySegmentation;
  
  
public slots:
  void btnCompute_clicked();
};

#endif
