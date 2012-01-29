#include <QApplication>
#include <QLabel>

class MyClass
{
public:
  MyClass() : label(new QLabel){}
  QScopedPointer<QLabel> label;
};

int main( int argc, char **argv )
{
  QApplication app(argc, argv);

  QScopedPointer<QLabel> label1(new QLabel);
  
  // Can't do this:
  // QScopedPointer<QLabel> label2;
  // label2 = new QLabel;

  // Do this instead
  QScopedPointer<QLabel> label2;
  label2.reset(new QLabel);

  MyClass myClass;

  return app.exec();
}
