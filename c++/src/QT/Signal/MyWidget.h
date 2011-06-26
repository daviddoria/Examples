#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QObject>

class MyWidget : public QObject
{
Q_OBJECT

public:
  MyWidget();

  void EmitSignal();

public slots:
  void myslot();

signals:
  void mysignal();

};

#endif