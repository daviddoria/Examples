// Can't do this - you cannot have GUI objects in the non-main thread!

#include <qapplication.h>
#include "form.h"

int main( int argc, char **argv )
{
  QApplication app(argc, argv);
  MainWindow window;

  window.show();
  return app.exec();
}
