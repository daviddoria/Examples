#include <QApplication>
#include <QLabel>

#include "form.h"

int main(int argc, char*argv[])
{
  QApplication app(argc, argv);

  QLabel label(&app); // can't use 'app', need a QObject

  return app.exec();
}