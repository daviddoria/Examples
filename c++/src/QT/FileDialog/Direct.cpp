#include <qapplication.h>
#include <qpushbutton.h>

#include "ui_main.h"

int main( int argc, char **argv )
{
  QApplication app(argc, argv);
  QMainWindow *widget = new QMainWindow;
  Ui::MainWindow ui;
  ui.setupUi(widget);
  
  QObject::connect( ui.pushButton, SIGNAL(clicked()), ui.label, SLOT(setText(const QString&)) );
  
  widget->show();
  return app.exec();
}
