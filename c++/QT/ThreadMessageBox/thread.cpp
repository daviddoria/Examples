#include "thread.h"

#include <QMessageBox>
#include <iostream>

void Thread::run()
{
  std::cout << "Thread::run()" << std::endl;
  emit DisplayMessageBoxSignal();
}