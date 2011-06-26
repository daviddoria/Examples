#ifndef THREAD_H
#define THREAD_H

#include <QThread>

#include "TestClass.h"

class Thread : public QThread
{
Q_OBJECT
public:
  void run();

signals:
  void DisplayMessageBoxSignal();

public:
  TestClass MyTestClass;
};

#endif