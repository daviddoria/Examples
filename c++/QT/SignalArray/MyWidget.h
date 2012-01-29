#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QObject>

class MyWidget : public QObject
{
Q_OBJECT

public:
  MyWidget();

public slots:
  void myslot();

};

#endif