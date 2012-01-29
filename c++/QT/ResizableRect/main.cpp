#include <QApplication>

#include "form.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  MainWindow form;

  form.show();
  return app.exec();
}
