#include "thread.h"

#include <QMessageBox>
#include <iostream>

void Thread::run()
{
  std::cout << "Message box from thread" << std::endl;

  QMessageBox msgBox;
  msgBox.setText("Test Text");
  msgBox.exec();

}