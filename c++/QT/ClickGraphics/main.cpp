#include <QApplication>

#include "form.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Form form;

  form.show();
  return app.exec();
}


/*
// works fine

#include <QtGui>

int main(int argc, char *argv[])
{
QApplication app(argc, argv);

QGraphicsView graphicsView;
QGraphicsScene scene;
graphicsView.setScene(&scene);

QFont font("Ariel", -1, -1, true);

scene.addText("Some text", font);

graphicsView.show();

return app.exec();
}
*/