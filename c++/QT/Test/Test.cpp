/*
 * #include <QApplication>

#include "ui_Test.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

	QMainWindow *window = new QMainWindow;
    Ui::MainWindow ui;
    ui.setupUi(window);

    window->show();
    return app.exec();
}

*/
//#include <QtGui/QApplication>
#include <QApplication>
#include <QPushButton>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  return a.exec();
}