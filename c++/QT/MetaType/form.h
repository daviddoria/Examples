#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

struct MyType
{
  float a;
};

struct ClassWithNoDefaultConstructor
{
public:
  ClassWithNoDefaultConstructor(int a){}
  float a;
};

struct OtherClass : public QObject
{
Q_OBJECT

signals:
  void MySignal(const MyType&);
  void MySignal(const ClassWithNoDefaultConstructor&);

public:
  void EmitSignal()
  {
    MyType a;
    emit MySignal(a);
  }
};

class MyForm : public QWidget, private Ui::Form
{
Q_OBJECT
public:
  MyForm(QWidget *parent = 0);

public slots:
  void on_btnButton_clicked();
  void MySlot(const MyType&);
  void MySlot(const ClassWithNoDefaultConstructor&);

protected:
  OtherClass otherClass;
};

#endif
