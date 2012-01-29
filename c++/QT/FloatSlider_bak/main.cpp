#include <QApplication>

#include "FloatSlider.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  FloatSlider slider;

  slider.show();
  return app.exec();
}
