#include <QApplication>
#include "SimpleViewUI.h"
 
int main( int argc, char** argv )
{
  // QT Stuff
  QApplication app( argc, argv );
 
  SimpleView mySimpleView;
  mySimpleView.show();
 
  return app.exec();
}
