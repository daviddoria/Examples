#include <QProgressDialog>
#include <QApplication>

#include "form.h"

int main( int argc, char **argv )
{
  QApplication app(argc, argv);
  MainWindow window;
  
  window.show();
  
  return app.exec();
}
