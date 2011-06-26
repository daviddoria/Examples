#include "thread.h"

#include <QMessageBox>
#include <iostream>

void Thread::run()
{
  MyTestClass.MyFunction();

  emit DisplayMessageBoxSignal();
}