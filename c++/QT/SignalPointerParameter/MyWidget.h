#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QObject>

class MyClass
{
public:
  int a;
};

class MyWidget : public QObject
{
Q_OBJECT

public:
  MyWidget();

  void EmitSignal();

public slots:
  void myslot(MyClass*);

signals:
  void mysignal(MyClass*);

};

#endif