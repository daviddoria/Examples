#include <QApplication>
#include <iostream>

#include "form.h"

#include <boost/signals2/signal.hpp>

void func()
{
  std::cout << "Hello, world!" << std::endl;
}

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  MyForm form;

  form.show();

  boost::signals2::signal<void ()> s;
  s.connect(func);

  return app.exec();
}
