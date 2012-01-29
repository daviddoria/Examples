/*
 * See 'Style' example for use of QIcon with QCommonStyle to use built in icons
 */

#include <QApplication>

#include "form.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Form form;
  
  form.show();
  return app.exec();
}
