// #include <QApplication>
// 
// #include "form.h"
// 
// int main(int argc, char *argv[])
// {
//   QApplication app(argc, argv);
//   Form form;
// 
//   form.show();
//   return app.exec();
// }

#include <QtGui>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QRubberBand band(QRubberBand::Rectangle);

  QPalette pal;
  pal.setBrush(QPalette::Highlight, QBrush(Qt::red));
  band.setPalette(pal);

  band.resize(30, 30);
  band.show();
  return app.exec();
}