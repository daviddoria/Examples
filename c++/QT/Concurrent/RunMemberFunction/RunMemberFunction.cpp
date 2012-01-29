#include <QApplication>
#include <QObject>
#include <QThread>

#include <iostream>

#include "form.h"

int main(int argc, char*argv[])
{
  QApplication app(argc, argv);

  Form form;

  form.show();

  return app.exec();
}