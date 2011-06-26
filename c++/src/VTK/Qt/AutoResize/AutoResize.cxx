/*
 * To create this, add the QVTKWidget to the form (in Qt Designer).
 * Right click the form, go to Layout, then choose Layout Horizontally and Layout Vertically
*/
#include <QApplication>
#include "Form.h"

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  Form mySimpleView;
  mySimpleView.show();

  return app.exec();
}
