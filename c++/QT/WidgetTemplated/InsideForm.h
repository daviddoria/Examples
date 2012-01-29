#ifndef INSIDEFORM_H
#define INSIDEFORM_H

#include "ui_InsideForm.h"

//class InsideFormObject : public QWidget, private Ui::InsideForm
class InsideFormObject : public QWidget, public Ui::InsideForm
{
Q_OBJECT
public:
    InsideFormObject(QWidget *parent = 0);

public slots:
    virtual void btnButton_clicked() = 0;
};

template <typename T>
class MyInsideForm : public InsideFormObject//, public Ui::InsideForm
{
public:
  void btnButton_clicked();

  MyInsideForm(QWidget *parent = 0);

  void SetValue(T temp) {this->value = temp;}
  T value;
};

#include "InsideForm.txx"
#endif
